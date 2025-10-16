#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>
#include <math.h>

#define CH                0           // TX channel A
#define NCO_INDEX         0
#define FIFO_SIZE_SAMPLES (1<<17)     // bigger FIFO for stability
#define BUF_SAMPLES       8192        // larger chunk reduces IRQ/USB churn
#define SEND_TIMEOUT_MS   1000
#define TONE_SCALE        0.70        // 70% FS to avoid DAC clipping
#define TX_GAIN_MIN_DB    0
#define TX_GAIN_MAX_DB    73          // Lime typical range

#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)

static volatile int keep_running = 1;
static void on_sigint(int s){ (void)s; keep_running = 0; }

static void print_sr(lms_device_t* dev){
    double host=0, rf=0;
    if (!LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host, &rf))
        printf("Set/Get: SampleRate host=%.2f Msps, rf=%.2f Msps\n", host/1e6, rf/1e6);
}

static void print_gain(lms_device_t* dev){
    unsigned int g = 0;
    if (!LMS_GetGaindB(dev, LMS_CH_TX, CH, &g))
        printf("Set/Get: TX Gain = %u dB\n", g);
}

static void print_lo(lms_device_t* dev){
    double lof = 0;
    if (!LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &lof))
        printf("Set/Get: LO = %.6f MHz\n", lof/1e6);
}

static void print_nco(lms_device_t* dev){
    int idx = LMS_GetNCOIndex(dev, true, CH);
    printf("Set/Get: NCO idx=%d (no frequency readback in this LimeSuite)\n", idx);
}

static void usage(const char* prog){
    fprintf(stderr,
        "Usage: %s [options]\n"
        "RF & DSP:\n"
        "  --host-sr <Hz>          Host sample rate (e.g., 5e6, 5M) [default 5M]\n"
        "  --oversample <N>        Oversample factor (int)          [default 32]\n"
        "  --tx-lpf-bw <Hz>        TX LPF bandwidth                 [default 20M]\n"
        "  --lo <Hz>               LO frequency                     [default 30M]\n"
        "  --nco <Hz>              NCO frequency (magnitude)        [default 15M]\n"
        "  --nco-downconvert <0|1|true|false>  RF=LO-NCO if true    [default true]\n"
        "\n"
        "Gain (smooth ramp):\n"
        "  --tx-gain-start <dB>    Starting TX gain                 [default 0]\n"
        "  --tx-gain <dB>          Target TX gain                   [default 40]\n"
        "  --gain-ramp-ms <ms>     Total ramp duration              [default 2000]\n"
        "  --gain-ramp-interval-ms <ms>  Step interval              [default 20]\n"
        "\n"
        "Misc:\n"
        "  -h, --help              Show this help\n\n", prog);
}

static bool parse_bool(const char* s, bool* out){
    if (!s || !out) return false;
    if (!strcasecmp(s,"1") || !strcasecmp(s,"true") || !strcasecmp(s,"yes") || !strcasecmp(s,"on")) { *out=true;  return true; }
    if (!strcasecmp(s,"0") || !strcasecmp(s,"false")|| !strcasecmp(s,"no")  || !strcasecmp(s,"off")){ *out=false; return true; }
    return false;
}

// Parse Hz with optional k/M/G suffix (case-insensitive). Returns true on success.
static bool parse_hz(const char* s, double* out){
    if (!s || !out) return false;
    char* end = NULL;
    double v = strtod(s, &end);
    if (end && *end != '\0'){
        while (*end && isspace((unsigned char)*end)) end++;
        if (*end){
            double mul = 1.0;
            char c = (char)tolower((unsigned char)*end);
            if (c=='k') mul = 1e3;
            else if (c=='m') mul = 1e6;
            else if (c=='g') mul = 1e9;
            else return false;
            end++;
            while (*end && isspace((unsigned char)*end)) end++;
            if (*end!='\0') return false;
            v *= mul;
        }
    }
    *out = v;
    return true;
}

static int clampi(int v, int lo, int hi){ if (v<lo) return lo; if (v>hi) return hi; return v; }

static uint64_t now_ms(void){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000ull + (uint64_t)(ts.tv_nsec/1000000ull);
}

int main(int argc, char** argv)
{
    // Defaults
    double HOST_SR_HZ      = 5e6;
    int    OVERSAMPLE      = 32;
    double TX_LPF_BW_HZ    = 20e6;
    double LO_HZ           = 30e6;
    double NCO_FREQ_HZ     = 15e6;
    bool   NCO_DOWNCONVERT = true;
    int    TX_GAIN_DB      = 40;   // target
    int    TX_GAIN_START   = 0;    // start
    int    RAMP_MS         = 2000; // total time
    int    RAMP_INTERVAL_MS= 20;   // step interval

    // Arg parsing
    for (int i=1; i<argc; i++){
        const char* a = argv[i];
        if (!strcmp(a,"-h") || !strcmp(a,"--help")) { usage(argv[0]); return 0; }

        #define NEEDVAL() do{ if (i+1>=argc){ fprintf(stderr,"Missing value for %s\n", a); usage(argv[0]); return 1; } }while(0)

        if (!strcmp(a,"--host-sr")){ NEEDVAL(); if(!parse_hz(argv[++i], &HOST_SR_HZ)) { fprintf(stderr,"Bad --host-sr\n"); return 1; } continue; }
        if (!strcmp(a,"--oversample")){ NEEDVAL(); OVERSAMPLE = (int)strtol(argv[++i], NULL, 0); if (OVERSAMPLE<1){ fprintf(stderr,"Bad --oversample\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-lpf-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &TX_LPF_BW_HZ)) { fprintf(stderr,"Bad --tx-lpf-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--lo")){ NEEDVAL(); if(!parse_hz(argv[++i], &LO_HZ)) { fprintf(stderr,"Bad --lo\n"); return 1; } continue; }
        if (!strcmp(a,"--nco")){ NEEDVAL(); if(!parse_hz(argv[++i], &NCO_FREQ_HZ)) { fprintf(stderr,"Bad --nco\n"); return 1; } continue; }
        if (!strcmp(a,"--nco-downconvert")){ NEEDVAL(); if(!parse_bool(argv[++i], &NCO_DOWNCONVERT)) { fprintf(stderr,"Bad --nco-downconvert\n"); return 1; } continue; }

        if (!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB = (int)strtol(argv[++i], NULL, 0); continue; }
        if (!strcmp(a,"--tx-gain-start")){ NEEDVAL(); TX_GAIN_START = (int)strtol(argv[++i], NULL, 0); continue; }
        if (!strcmp(a,"--gain-ramp-ms")){ NEEDVAL(); RAMP_MS = (int)strtol(argv[++i], NULL, 0); continue; }
        if (!strcmp(a,"--gain-ramp-interval-ms")){ NEEDVAL(); RAMP_INTERVAL_MS = (int)strtol(argv[++i], NULL, 0); continue; }

        fprintf(stderr,"Unknown option: %s\n", a);
        usage(argv[0]);
        return 1;
    }

    // Clamp gains
    TX_GAIN_DB    = clampi(TX_GAIN_DB,    TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);
    TX_GAIN_START = clampi(TX_GAIN_START, TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);
    if (RAMP_INTERVAL_MS < 1) RAMP_INTERVAL_MS = 1;
    if (RAMP_MS < 0) RAMP_MS = 0;

    lms_device_t* dev = NULL;
    lms_stream_t  txs;
    int16_t*      buf = NULL;

    signal(SIGINT, on_sigint);

    // 1) Open device
    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) { fprintf(stderr,"No LimeSDR found\n"); return 1; }
    if (LMS_Open(&dev, list[0], NULL)) { fprintf(stderr,"LMS_Open failed: %s\n", LMS_GetLastErrorMessage()); return 1; }

    // 2) Basic setup
    CHECK(LMS_Init(dev));
    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    printf("TX channel enabled.\n");

    // Sample rates
    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    print_sr(dev);

    // TX LPF/BW
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));

    // Set initial gain (start of ramp)
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_START));
    print_gain(dev);

    // LO
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));
    print_lo(dev);

    // Optional: Calibrate near operating BW
    CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ, 0));

    // 3) NCO
    {
        double freqs[16]={0};
        freqs[NCO_INDEX] = NCO_FREQ_HZ; // magnitude; direction set below
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));              // dir_tx=true
        CHECK(LMS_SetNCOIndex    (dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
        int idx = LMS_GetNCOIndex(dev, true, CH);
        if (idx < 0) { fprintf(stderr,"LMS_GetNCOIndex failed: %s\n", LMS_GetLastErrorMessage()); goto cleanup; }
        print_nco(dev);
    }

    // 4) TX stream
    memset(&txs, 0, sizeof(txs));
    txs.channel  = CH;
    txs.isTx     = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt  = LMS_FMT_I16; // C-safe
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16).\n", FIFO_SIZE_SAMPLES);

    // Constant DC IQ -> pure RF tone after NCO
    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf) { fprintf(stderr,"malloc failed\n"); goto cleanup; }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0);
    const int16_t Q = 0;
    for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=I; buf[2*i+1]=Q; }

    double host_sr=0, rf_sr=0;
    LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr); // best-effort

    const double rf_hz = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
    printf("TX @ %.6f MHz  (host=%.2f Msps, rf=%.2f Msps, start_gain=%d dB -> target=%d dB, ramp=%d ms, step=%d ms, %sconvert).\n",
           rf_hz/1e6, host_sr/1e6, rf_sr/1e6, TX_GAIN_START, TX_GAIN_DB, RAMP_MS, RAMP_INTERVAL_MS,
           NCO_DOWNCONVERT?"down":"up");
    printf("Ctrl+C to stop.\n");

    // 5) Stream loop + gain ramp
    const bool use_ramp = (RAMP_MS > 0) && (TX_GAIN_DB != TX_GAIN_START);
    const int  total_delta = TX_GAIN_DB - TX_GAIN_START; // may be negative if ever allowed
    const int  steps = use_ramp ? (RAMP_MS + RAMP_INTERVAL_MS - 1) / RAMP_INTERVAL_MS : 1;
    const double step_db_f = use_ramp ? ((double)total_delta / (double)steps) : 0.0;

    uint64_t t0 = now_ms();
    uint64_t t_next = t0 + (use_ramp ? (uint64_t)RAMP_INTERVAL_MS : UINT64_MAX);
    double   g_accum = (double)TX_GAIN_START; // track fractional dB
    int      g_last_applied = TX_GAIN_START;

    while (keep_running) {
        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }

        // ramp handler
        if (use_ramp) {
            uint64_t now = now_ms();
            while (now >= t_next && keep_running) {
                g_accum += step_db_f;
                int g_int = clampi((int)llround(g_accum), TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);

                // Avoid redundant I2C/SPI if unchanged
                if (g_int != g_last_applied) {
                    if (LMS_SetGaindB(dev, LMS_CH_TX, CH, g_int) == 0) {
                        g_last_applied = g_int;
                        // Optional: uncomment to see the ramp progress
                        // printf("Gain -> %d dB\n", g_int);
                    } else {
                        fprintf(stderr,"Gain ramp set failed: %s\n", LMS_GetLastErrorMessage());
                    }
                }

                t_next += (uint64_t)RAMP_INTERVAL_MS;

                // Done?
                if ((step_db_f >= 0 && g_last_applied >= TX_GAIN_DB) ||
                    (step_db_f <  0 && g_last_applied <= TX_GAIN_DB)) {
                    // Snap exactly to target once and stop ramping
                    if (g_last_applied != TX_GAIN_DB) {
                        (void)LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB);
                        g_last_applied = TX_GAIN_DB;
                    }
                    t_next = UINT64_MAX; // no more ramp ticks
                    break;
                }
                now = now_ms();
            }
        }
    }

    printf("\nSIGINT detected: muting TX and shutting down safely...\n");

cleanup:
    // Graceful mute: push one zero buffer so the last burst is silence
    if (txs.handle) {
        int16_t* z = (int16_t*)calloc(2*BUF_SAMPLES, sizeof(int16_t));
        if (z){
            lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
            (void)LMS_SendStream(&txs, z, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS);
            free(z);
        }
        LMS_StopStream(&txs);
        LMS_DestroyStream(dev, &txs);
        printf("TX stream stopped.\n");
    }

    // Disable TX channel (safe TX off)
    if (dev) {
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled.\n");
    }

    if (buf) free(buf);
    if (dev) LMS_Close(dev);
    return 0;
}
