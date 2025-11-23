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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CH                0
#define NCO_INDEX         0
#define FIFO_SIZE_SAMPLES (1<<17)
#define BUF_SAMPLES       8192
#define SEND_TIMEOUT_MS   1000
#define TONE_SCALE_DEF    0.70
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
    printf("Set/Get: NCO idx=%d (frequency printed from requested value)\n", idx);
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
        "  --set-gain-i <0..2047>        Manually set GCORRI (I gain)\n"
        "  --set-gain-q <0..2047>        Manually set GCORRQ (Q gain)\n"
        "  --set-phase  <-2047..2047>    Manually set IQCORR (phase)\n"
        "  --set-dc-i   <-128..127>      Manually set DCCORRI (I DC)\n"
        "  --set-dc-q   <-128..127>      Manually set DCCORRQ (Q DC)\n"
        "\n"
        "Misc:\n"
        "  -h, --help              Show this help\n\n", prog);
}

static int set_mac_channel(lms_device_t* dev, int ch){
    uint16_t v = 0;
    if (LMS_ReadLMSReg(dev, 0x0020, &v)) return -1;
    v = (uint16_t)((v & ~0x3) | (ch==0 ? 0x1 : 0x2));
    return LMS_WriteLMSReg(dev, 0x0020, v);
}
static void print_tx_correctors(lms_device_t* dev, int ch){
    if (set_mac_channel(dev, ch)) {
        fprintf(stderr, "WARN: can't set MAC for channel %c\n", ch? 'B':'A');
        return;
    }
    uint16_t reg_gq=0, reg_gi=0, reg_iq=0, reg_dc=0, reg_byp=0;
    (void)LMS_ReadLMSReg(dev, 0x0201, &reg_gq);
    (void)LMS_ReadLMSReg(dev, 0x0202, &reg_gi);
    (void)LMS_ReadLMSReg(dev, 0x0203, &reg_iq);
    (void)LMS_ReadLMSReg(dev, 0x0204, &reg_dc);
    (void)LMS_ReadLMSReg(dev, 0x0208, &reg_byp);

    int gq_u   = (int)(reg_gq & 0x07FF);
    int gi_u   = (int)(reg_gi & 0x07FF);
    int16_t iq_s = (int16_t)(reg_iq & 0x0FFF); if (iq_s & 0x0800) iq_s |= 0xF000;
    int8_t dci_s = (int8_t)((reg_dc >> 8) & 0xFF);
    int8_t dcq_s = (int8_t)(reg_dc & 0xFF);

    const double gq = gq_u / 2048.0;
    const double gi = gi_u / 2048.0;
    const double gq_db = 20.0*log10(gq>0?gq:1e-9);
    const double gi_db = 20.0*log10(gi>0?gi:1e-9);

    const double tan_half = iq_s / 2048.0;
    const double alpha_rad = 2.0 * atan(tan_half);
    const double alpha_deg = alpha_rad * 180.0 / M_PI;

    const double dci_norm = dci_s / 128.0;
    const double dcq_norm = dcq_s / 128.0;

    const bool ph_bypass = (reg_byp & (1u<<0)) != 0;
    const bool gc_bypass = (reg_byp & (1u<<1)) != 0;
    const bool dc_bypass = (reg_byp & (1u<<3)) != 0;

    printf("TXTSP correctors (CH %c):\n", ch? 'B':'A');
    printf("  Gain:   GCORRI=%4d  (%.6f, %+6.2f dB)%s,  GCORRQ=%4d  (%.6f, %+6.2f dB)%s\n",
           gi_u, gi, gi_db, gc_bypass?" [BYPASSED]":"",
           gq_u, gq, gq_db, gc_bypass?" [BYPASSED]":"");
    printf("  Phase:  IQCORR=%5d  -> phase ≈ %+8.4f deg%s\n",
           (int)iq_s, alpha_deg, ph_bypass?" [BYPASSED]":"");
    printf("  DC:     DCCORRI=%4d (%.5f FS)%s,  DCCORRQ=%4d (%.5f FS)%s\n",
           (int)dci_s, dci_norm, dc_bypass?" [BYPASSED]":"",
           (int)dcq_s, dcq_norm, dc_bypass?" [BYPASSED]":"");
}

static void print_snapshot(lms_device_t* dev,
                           const char* title,
                           double requested_tx_lpf_bw_hz,
                           double requested_nco_hz,
                           bool     nco_downconvert,
                           double   tone_scale)
{
    double host_sr=0, rf_sr=0, lo=0;
    LMS_GetSampleRate  (dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    LMS_GetLOFrequency (dev, LMS_CH_TX, CH, &lo);
    unsigned int gdb=0; LMS_GetGaindB(dev, LMS_CH_TX, CH, &gdb);
    int nco_idx = LMS_GetNCOIndex(dev, true, CH);

    const double rf_hz = nco_downconvert ? (lo - requested_nco_hz) : (lo + requested_nco_hz);

    printf("\n=== %s ===\n", title);
    printf(" Host SR      : %.6f Msps\n", host_sr/1e6);
    printf(" RF SR        : %.6f Msps\n", rf_sr/1e6);
    printf(" TX LPF BW    : %.3f MHz (requested)\n", requested_tx_lpf_bw_hz/1e6);
    printf(" LO           : %.6f MHz (get)\n", lo/1e6);
    printf(" NCO idx/dir  : %d / %s\n", nco_idx, nco_downconvert?"downconvert (RF=LO-NCO)":"upconvert (RF=LO+NCO)");
    printf(" NCO freq     : %.6f MHz (requested)\n", requested_nco_hz/1e6);
    printf(" Target RF    : %.6f MHz (computed from LO±NCO)\n", rf_hz/1e6);
    printf(" TX Gain (dB) : %u (current)\n", gdb);
    printf(" Tone scale   : %.2f (fraction of full-scale)\n", tone_scale);
    print_tx_correctors(dev, CH);
    printf("=============================================================\n");
}

static int apply_manual_txtsp(lms_device_t* dev, int ch,
                              bool have_gi,    int gi,
                              bool have_gq,    int gq,
                              bool have_phase, int phase,
                              bool have_dci,   int dci,
                              bool have_dcq,   int dcq)
{
    if (set_mac_channel(dev, ch)) return -1;

    int rc = 0;

    if (have_gi) {
        gi = clampi(gi, 0, 2047);
        rc |= LMS_WriteLMSReg(dev, 0x0202, (uint16_t)((uint16_t)gi & 0x07FF));
    }
    if (have_gq) {
        gq = clampi(gq, 0, 2047);
        rc |= LMS_WriteLMSReg(dev, 0x0201, (uint16_t)((uint16_t)gq & 0x07FF));
    }
    if (have_phase) {
        phase = clampi(phase, -2047, 2047);
        uint16_t iq12 = (uint16_t)((uint16_t)phase & 0x0FFF);
        rc |= LMS_WriteLMSReg(dev, 0x0203, iq12);
    }

    if (have_dci || have_dcq) {
        uint16_t reg_dc = 0;
        rc |= LMS_ReadLMSReg(dev, 0x0204, &reg_dc);
        if (have_dci) {
            int8_t dci_s = (int8_t)clampi(dci, -128, 127);
            reg_dc = (uint16_t)((reg_dc & 0x00FFu) | ((uint16_t)(uint8_t)dci_s << 8));
        }
        if (have_dcq) {
            int8_t dcq_s = (int8_t)clampi(dcq, -128, 127);
            reg_dc = (uint16_t)((reg_dc & 0xFF00u) | (uint16_t)(uint8_t)dcq_s);
        }
        rc |= LMS_WriteLMSReg(dev, 0x0204, reg_dc);
    }

    uint16_t byp = 0;
    rc |= LMS_ReadLMSReg(dev, 0x0208, &byp);
    byp &= (uint16_t)~((1u<<0) | (1u<<1) | (1u<<3));
    rc |= LMS_WriteLMSReg(dev, 0x0208, byp);

    return rc;
}

int main(int argc, char** argv)
{
    double HOST_SR_HZ      = 5e6;
    int    OVERSAMPLE      = 32;
    double TX_LPF_BW_HZ    = 20e6;
    double LO_HZ           = 30e6;
    double NCO_FREQ_HZ     = 15e6;
    bool   NCO_DOWNCONVERT = true;
    int    TX_GAIN_DB      = 40;
    int    TX_GAIN_START   = 0;
    int    RAMP_MS         = 2000;
    int    RAMP_INTERVAL_MS= 20;
    double TONE_SCALE      = TONE_SCALE_DEF;
    bool   DO_CAL          = false;

    bool SET_GI=false, SET_GQ=false, SET_PHASE=false, SET_DCI=false, SET_DCQ=false;
    int  MAN_GI=0,     MAN_GQ=0,     MAN_PHASE=0,     MAN_DCI=0,     MAN_DCQ=0;

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

        if (!strcmp(a,"--set-gain-i")){ NEEDVAL(); SET_GI=true;    MAN_GI    = (int)strtol(argv[++i], NULL, 0); MAN_GI    = clampi(MAN_GI,    0, 2047); continue; }
        if (!strcmp(a,"--set-gain-q")){ NEEDVAL(); SET_GQ=true;    MAN_GQ    = (int)strtol(argv[++i], NULL, 0); MAN_GQ    = clampi(MAN_GQ,    0, 2047); continue; }
        if (!strcmp(a,"--set-phase")) { NEEDVAL(); SET_PHASE=true; MAN_PHASE = (int)strtol(argv[++i], NULL, 0); MAN_PHASE = clampi(MAN_PHASE,-2047, 2047); continue; }
        if (!strcmp(a,"--set-dc-i"))  { NEEDVAL(); SET_DCI=true;   MAN_DCI   = (int)strtol(argv[++i], NULL, 0); MAN_DCI   = clampi(MAN_DCI, -128, 127); continue; }
        if (!strcmp(a,"--set-dc-q"))  { NEEDVAL(); SET_DCQ=true;   MAN_DCQ   = (int)strtol(argv[++i], NULL, 0); MAN_DCQ   = clampi(MAN_DCQ, -128, 127); continue; }

        fprintf(stderr,"Unknown option: %s\n", a);
        usage(argv[0]);
        return 1;
    }

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

    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) { fprintf(stderr,"No LimeSDR found\n"); return 1; }
    if (LMS_Open(&dev, list[0], NULL)) { fprintf(stderr,"LMS_Open failed: %s\n", LMS_GetLastErrorMessage()); return 1; }

    CHECK(LMS_Init(dev));

    CHECK(LMS_Reset(dev));
    printf("device reset to defaults\n");

    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    printf("TX channel enabled.\n");

    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    print_sr(dev);

    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));

    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_START));
    print_gain(dev);

    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));
    print_lo(dev);

    {
        double freqs[16]={0};
        freqs[NCO_INDEX] = NCO_FREQ_HZ;
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
        CHECK(LMS_SetNCOIndex    (dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
        print_nco(dev);
    }

    print_snapshot(dev, DO_CAL ? "BEFORE calibration" : "Parameters (calibration OFF)",
                   TX_LPF_BW_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT, TONE_SCALE);

    int calib_rc = 0;
    if (DO_CAL) {
        printf("Running LMS_Calibrate(TX ch=%d, bw=%.3f MHz)...\n", CH, TX_LPF_BW_HZ/1e6);
        calib_rc = LMS_Calibrate(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ, 0);
        if (calib_rc) fprintf(stderr,"LMS_Calibrate returned %d: %s\n", calib_rc, LMS_GetLastErrorMessage());
        else          printf("Calibration OK.\n");

        print_snapshot(dev, "AFTER calibration",
                       TX_LPF_BW_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT, TONE_SCALE);
    }

    if (SET_GI || SET_GQ || SET_PHASE || SET_DCI || SET_DCQ) {
        CHECK(apply_manual_txtsp(dev, CH,
                                 SET_GI, MAN_GI,
                                 SET_GQ, MAN_GQ,
                                 SET_PHASE, MAN_PHASE,
                                 SET_DCI, MAN_DCI,
                                 SET_DCQ, MAN_DCQ));
        print_snapshot(dev, DO_CAL ? "AFTER manual correctors (override calibration)" : "AFTER manual correctors",
                       TX_LPF_BW_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT, TONE_SCALE);
    }

    memset(&txs, 0, sizeof(txs));
    txs.channel  = CH;
    txs.isTx     = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt  = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16).\n", FIFO_SIZE_SAMPLES);

    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf) { fprintf(stderr,"malloc failed\n"); goto cleanup; }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0);
    const int16_t Q = 0;
    for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=I; buf[2*i+1]=Q; }

    double host_sr=0, rf_sr=0; LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    double lo_now=0; LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &lo_now);
    const double rf_hz = NCO_DOWNCONVERT ? (lo_now - NCO_FREQ_HZ) : (lo_now + NCO_FREQ_HZ);
    printf("TX RF sine @ %.6f MHz  (host=%.2f Msps, rf=%.2f Msps, start_gain=%d dB -> target=%d dB, ramp=%d ms, step=%d ms, %sconvert).\n",
           rf_hz/1e6, host_sr/1e6, rf_sr/1e6, TX_GAIN_START, TX_GAIN_DB, RAMP_MS, RAMP_INTERVAL_MS,
           NCO_DOWNCONVERT?"down":"up");
    printf("Ctrl+C to stop.\n");

    const bool use_ramp = (RAMP_MS > 0) && (TX_GAIN_DB != TX_GAIN_START);
    const int  total_delta = TX_GAIN_DB - TX_GAIN_START;
    const int  steps = use_ramp ? (RAMP_MS + RAMP_INTERVAL_MS - 1) / RAMP_INTERVAL_MS : 1;
    const double step_db_f = use_ramp ? ((double)total_delta / (double)steps) : 0.0;

    uint64_t t0 = now_ms();
    uint64_t t_next = t0 + (use_ramp ? (uint64_t)RAMP_INTERVAL_MS : UINT64_MAX);
    double   g_accum = (double)TX_GAIN_START;
    int      g_last_applied = TX_GAIN_START;

    while (keep_running) {
        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }

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
    if (dev) {
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled.\n");
        LMS_Close(dev);
    }
    if (buf) free(buf);
    return 0;
}
