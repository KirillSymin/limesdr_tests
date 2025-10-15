// tx_sine_nco.c
// Generate an RF sine using Lime's TX NCO (DC IQ -> NCO -> single tone at LO ± NCO).
// Usage example:
//   ./tx_sine_nco --host-sr 5M --oversample 32 --tx-lpf-bw 20M \
//                 --lo 30M --nco 15M --nco-downconvert true \
//                 --tx-gain-start 0 --tx-gain 40 --gain-ramp-ms 2000 \
//                 --calibrate false
//
// Build: gcc tx_sine_nco.c -o tx_sine_nco -lLimeSuite -lm

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
#define FIFO_SIZE_SAMPLES (1<<17)     // big FIFO for smooth streaming
#define BUF_SAMPLES       8192        // larger chunk reduces USB/IRQ churn
#define SEND_TIMEOUT_MS   1000
#define TONE_SCALE_DEF    0.70        // 70% FS to avoid DAC clipping
#define TX_GAIN_MIN_DB    0
#define TX_GAIN_MAX_DB    73

#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)

static volatile int keep_running = 1;
static void on_sigint(int s){ (void)s; keep_running = 0; }

static uint64_t now_ms(void){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000ull + (uint64_t)(ts.tv_nsec/1000000ull);
}
static int clampi(int v, int lo, int hi){ if (v<lo) return lo; if (v>hi) return hi; return v; }

static bool parse_bool(const char* s, bool* out){
    if (!s || !out) return false;
    if (!strcasecmp(s,"1")||!strcasecmp(s,"true") ||!strcasecmp(s,"yes")||!strcasecmp(s,"on"))  { *out=true;  return true; }
    if (!strcasecmp(s,"0")||!strcasecmp(s,"false")||!strcasecmp(s,"no") ||!strcasecmp(s,"off")) { *out=false; return true; }
    return false;
}
// Parse Hz with optional k/M/G suffix (case-insensitive). Returns true on success.
static bool parse_hz(const char* s, double* out){
    if (!s || !out) return false;
    char* end=NULL; double v=strtod(s,&end);
    if (end && *end!='\0'){
        while (*end && isspace((unsigned char)*end)) end++;
        if (*end){
            double mul=1.0; char c=(char)tolower((unsigned char)*end);
            if (c=='k') mul=1e3; else if (c=='m') mul=1e6; else if (c=='g') mul=1e9; else return false;
            end++;
            while (*end && isspace((unsigned char)*end)) end++;
            if (*end!='\0') return false;
            v*=mul;
        }
    }
    *out=v; return true;
}

static void print_sr(lms_device_t* dev){
    double host=0, rf=0;
    if (!LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host, &rf))
        printf("Set/Get: SampleRate host=%.2f Msps, rf=%.2f Msps\n", host/1e6, rf/1e6);
}
static void print_gain(lms_device_t* dev){
    unsigned int g=0;
    if (!LMS_GetGaindB(dev, LMS_CH_TX, CH, &g))
        printf("Set/Get: TX Gain = %u dB\n", g);
}
static void print_lo(lms_device_t* dev){
    double lof=0;
    if (!LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &lof))
        printf("Set/Get: LO = %.6f MHz\n", lof/1e6);
}
static void print_nco(lms_device_t* dev){
    int idx=LMS_GetNCOIndex(dev, true, CH);
    printf("Set/Get: NCO idx=%d (frequency printed from user setting below)\n", idx);
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
        "Tone:\n"
        "  --tone-scale <0..1>     Baseband DC amplitude fraction   [default 0.70]\n"
        "\n"
        "Calibration:\n"
        "  --calibrate <0|1|true|false>  Run LMS_Calibrate(TX)      [default false]\n"
        "\n"
        "Misc:\n"
        "  -h, --help              Show this help\n\n", prog);
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
    int    TX_GAIN_START   = 0;    // ramp start
    int    RAMP_MS         = 2000; // total ramp time
    int    RAMP_INTERVAL_MS= 20;   // ramp step
    double TONE_SCALE      = TONE_SCALE_DEF;
    bool   DO_CAL          = false;

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

        if (!strcmp(a,"--tone-scale")){ NEEDVAL(); TONE_SCALE = strtod(argv[++i], NULL); continue; }

        if (!strcmp(a,"--calibrate")){ NEEDVAL(); if(!parse_bool(argv[++i], &DO_CAL)) { fprintf(stderr,"Bad --calibrate\n"); return 1; } continue; }

        fprintf(stderr,"Unknown option: %s\n", a);
        usage(argv[0]);
        return 1;
    }

    // Clamp/check
    TX_GAIN_DB    = clampi(TX_GAIN_DB,    TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);
    TX_GAIN_START = clampi(TX_GAIN_START, TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);
    if (RAMP_INTERVAL_MS < 1) RAMP_INTERVAL_MS = 1;
    if (RAMP_MS < 0) RAMP_MS = 0;
    if (TONE_SCALE < 0.0) TONE_SCALE = 0.0;
    if (TONE_SCALE > 1.0) TONE_SCALE = 1.0;

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

    // Optional: Calibration (OFF by default)
    int calib_rc = 0;
    if (DO_CAL) {
        printf("Running LMS_Calibrate(TX ch=%d, bw=%.3f MHz)...\n", CH, TX_LPF_BW_HZ/1e6);
        calib_rc = LMS_Calibrate(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ, 0);
        if (calib_rc) fprintf(stderr,"LMS_Calibrate returned %d: %s\n", calib_rc, LMS_GetLastErrorMessage());
        else          printf("Calibration OK.\n");
    } else {
        printf("Calibration skipped (use --calibrate true to enable).\n");
    }

    // 3) NCO (this makes the RF sine)
    {
        double freqs[16]={0};
        freqs[NCO_INDEX] = NCO_FREQ_HZ; // magnitude; sign comes from direction flag below
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));              // dir_tx=true
        CHECK(LMS_SetNCOIndex    (dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
        print_nco(dev);
    }

    // --------- ALWAYS print effective / "calibrated" params block ----------
    {
        double host_sr=0, rf_sr=0, lo=0;
        LMS_GetSampleRate   (dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
        LMS_GetLOFrequency  (dev, LMS_CH_TX, CH, &lo);
        unsigned int gdb=0; LMS_GetGaindB(dev, LMS_CH_TX, CH, &gdb);
        const double rf_hz = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
        int nco_idx = LMS_GetNCOIndex(dev, true, CH);

        printf("\n=== Effective TX parameters ===\n");
        printf(" Calibration: %s (rc=%d)\n", DO_CAL ? "ENABLED (just ran)" : "DISABLED", calib_rc);
        printf(" Host SR      : %.6f Msps\n", host_sr/1e6);
        printf(" RF SR        : %.6f Msps\n", rf_sr/1e6);
        printf(" TX LPF BW    : %.3f MHz (requested)\n", TX_LPF_BW_HZ/1e6);
        printf(" LO           : %.6f MHz (get)\n", lo/1e6);
        printf(" NCO idx/dir  : %d / %s\n", nco_idx, NCO_DOWNCONVERT?"downconvert (RF=LO-NCO)":"upconvert (RF=LO+NCO)");
        printf(" NCO freq     : %.6f MHz (requested)\n", NCO_FREQ_HZ/1e6);
        printf(" Target RF    : %.6f MHz (computed)\n", rf_hz/1e6);
        printf(" TX Gain (dB) : %u (current)\n", gdb);
        printf(" Tone scale   : %.2f (fraction of full-scale)\n", TONE_SCALE);
        printf("================================================================\n\n");
    }
    // -----------------------------------------------------------------------

    // 4) TX stream
    lms_stream_t s; memset(&s, 0, sizeof(s));
    s.channel  = CH;
    s.isTx     = true;
    s.fifoSize = FIFO_SIZE_SAMPLES;
    s.dataFmt  = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &s));
    CHECK(LMS_StartStream(&s));
    printf("TX stream started (fifo=%d samples, fmt=I16).\n", FIFO_SIZE_SAMPLES);

    // Baseband: constant DC IQ -> the NCO turns this into an RF sine at LO ± NCO.
    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf) { fprintf(stderr,"malloc failed\n"); goto cleanup; }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0);
    const int16_t Q = 0;
    for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=I; buf[2*i+1]=Q; }

    double host_sr=0, rf_sr=0; LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);

    const double rf_hz = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
    printf("TX RF sine @ %.6f MHz  (host=%.2f Msps, rf=%.2f Msps, start_gain=%d dB -> target=%d dB, ramp=%d ms, step=%d ms, %sconvert).\n",
           rf_hz/1e6, host_sr/1e6, rf_sr/1e6, TX_GAIN_START, TX_GAIN_DB, RAMP_MS, RAMP_INTERVAL_MS,
           NCO_DOWNCONVERT?"down":"up");
    printf("Ctrl+C to stop.\n");

    // 5) Stream loop + gain ramp
    const bool use_ramp = (RAMP_MS > 0) && (TX_GAIN_DB != TX_GAIN_START);
    const int  total_delta = TX_GAIN_DB - TX_GAIN_START;
    const int  steps = use_ramp ? (RAMP_MS + RAMP_INTERVAL_MS - 1) / RAMP_INTERVAL_MS : 1;
    const double step_db_f = use_ramp ? ((double)total_delta / (double)steps) : 0.0;

    uint64_t t0 = now_ms();
    uint64_t t_next = t0 + (use_ramp ? (uint64_t)RAMP_INTERVAL_MS : UINT64_MAX);
    double   g_accum = (double)TX_GAIN_START; // track fractional dB
    int      g_last_applied = TX_GAIN_START;

    while (keep_running) {
        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if (LMS_SendStream(&s, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }

        // gain ramp handler
        if (use_ramp) {
            uint64_t now = now_ms();
            while (now >= t_next && keep_running) {
                g_accum += step_db_f;
                int g_int = clampi((int)llround(g_accum), TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);

                if (g_int != g_last_applied) {
                    if (LMS_SetGaindB(dev, LMS_CH_TX, CH, g_int) == 0) {
                        g_last_applied = g_int;
                    } else {
                        fprintf(stderr,"Gain ramp set failed: %s\n", LMS_GetLastErrorMessage());
                    }
                }
                t_next += (uint64_t)RAMP_INTERVAL_MS;

                if ((step_db_f >= 0 && g_last_applied >= TX_GAIN_DB) ||
                    (step_db_f <  0 && g_last_applied <= TX_GAIN_DB)) {
                    if (g_last_applied != TX_GAIN_DB) {
                        (void)LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB);
                        g_last_applied = TX_GAIN_DB;
                    }
                    t_next = UINT64_MAX;
                    break;
                }
                now = now_ms();
            }
        }
    }

    printf("\nSIGINT detected: muting TX and shutting down safely...\n");

cleanup:
    // Graceful mute: push one zero buffer so the last burst is silence
    if (s.handle) {
        int16_t* z = (int16_t*)calloc(2*BUF_SAMPLES, sizeof(int16_t));
        if (z){
            lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
            (void)LMS_SendStream(&s, z, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS);
            free(z);
        }
        LMS_StopStream(&s);
        LMS_DestroyStream(dev, &s);
        printf("TX stream stopped.\n");
    }
    if (dev) {
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled.\n");
        LMS_Close(dev);
    }
    if (buf) free(buf);
    return 0;
}
