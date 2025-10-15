// tx_sine_nco_calreset.c
// Generate an RF sine via Lime TX NCO (DC IQ -> NCO -> tone at LO ± NCO).
// Calibration flow:
//   - If --calibrate true: LMS_Reset() to defaults, then re-apply settings,
//     print BEFORE snapshot, run LMS_Calibrate(TX only), then print AFTER snapshot.
//   - If --calibrate false: configure once, print single snapshot (no reset).
//
// Build: gcc tx_sine_nco_calreset.c -o tx_sine_nco_calreset -lLimeSuite -lm

#include "lime/LimeSuite.h"
#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CH 0
#define NCO_INDEX 0
#define FIFO_SIZE_SAMPLES (1 << 17)
#define BUF_SAMPLES 8192
#define SEND_TIMEOUT_MS 1000
#define TONE_SCALE 0.70     // 70% FS to avoid DAC clipping

// clang-format off
#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)
// clang-format on

static volatile int keep_running = 1;
static void on_sigint(int s) {
    (void)s; keep_running = 0;
}

static void print_sr(lms_device_t *dev) {
    double host = 0, rf = 0;
    if (!LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host, &rf))
        printf("set/get: sample rate host=%.2f Msps, rf=%.2f Msps\n", host / 1e6, rf / 1e6);
}
static void print_gain(lms_device_t *dev) {
    unsigned int g = 0;
    if (!LMS_GetGaindB(dev, LMS_CH_TX, CH, &g))
        printf("set/get: TX gain = %u dB\n", g);
}
static void print_lo(lms_device_t *dev) {
    double lof = 0;
    if (!LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &lof))
        printf("set/get: LO freq = %.6f MHz\n", lof / 1e6);
}
static void print_nco(lms_device_t *dev) {
    int idx = LMS_GetNCOIndex(dev, true, CH);
    printf("set/get: NCO idx=%d (no frequency readback in this LimeSuite)\n", idx);
}

// ---------- MAC select + TXTSP correctors dump (as provided) ----------
static int set_mac_channel(lms_device_t* dev, int ch /*0=A,1=B*/){
    uint16_t v = 0;
    if (LMS_ReadLMSReg(dev, 0x0020, &v)) return -1;
    // MAC[1:0]: 01=A, 10=B
    v = (uint16_t)((v & ~0x3) | (ch==0 ? 0x1 : 0x2));
    return LMS_WriteLMSReg(dev, 0x0020, v);
}
static void print_tx_correctors(lms_device_t* dev, int ch /*0=A,1=B*/){
    if (set_mac_channel(dev, ch)) {
        fprintf(stderr, "WARN: can't set MAC for channel %c\n", ch? 'B':'A');
        return;
    }
    uint16_t reg_gq=0, reg_gi=0, reg_iq=0, reg_dc=0, reg_byp=0;
    (void)LMS_ReadLMSReg(dev, 0x0201, &reg_gq); // GCORRQ[10:0]
    (void)LMS_ReadLMSReg(dev, 0x0202, &reg_gi); // GCORRI[10:0]
    (void)LMS_ReadLMSReg(dev, 0x0203, &reg_iq); // IQCORR[11:0] (tan(alpha/2))
    (void)LMS_ReadLMSReg(dev, 0x0204, &reg_dc); // DCCORRI[15:8], DCCORRQ[7:0]
    (void)LMS_ReadLMSReg(dev, 0x0208, &reg_byp); // bypass flags

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

    const bool ph_bypass = (reg_byp & (1u<<0)) != 0; // PH_BYP
    const bool gc_bypass = (reg_byp & (1u<<1)) != 0; // GC_BYP
    const bool dc_bypass = (reg_byp & (1u<<3)) != 0; // DC_BYP

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
// ---------------------------------------------------------------------

static bool parse_bool(const char* s, bool* out){
    if (!s || !out) return false;
    if (!strcasecmp(s,"1") || !strcasecmp(s,"true") || !strcasecmp(s,"yes") || !strcasecmp(s,"on")) { *out=true;  return true; }
    if (!strcasecmp(s,"0") || !strcasecmp(s,"false")|| !strcasecmp(s,"no")  || !strcasecmp(s,"off")){ *out=false; return true; }
    return false;
}
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

static void print_snapshot(lms_device_t* dev,
                           const char* title,
                           double requested_tx_lpf_bw_hz,
                           double requested_nco_hz,
                           bool nco_downconvert)
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
    print_tx_correctors(dev, CH);
    printf("=============================================================\n");
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
    int    TX_GAIN_DB      = 40;
    double CAL_BW_HZ       = -1;
    bool   DO_CAL          = false;

    // Args
    for (int i=1; i<argc; i++){
        const char* a = argv[i];
        #define NEEDVAL() do{ if (i+1>=argc){ fprintf(stderr,"missing value for %s\n", a); return 1; } }while(0)

        if (!strcmp(a,"--host-sr")){ NEEDVAL(); if(!parse_hz(argv[++i], &HOST_SR_HZ)) { fprintf(stderr,"bad --host-sr\n"); return 1; } continue; }
        if (!strcmp(a,"--oversample")){ NEEDVAL(); OVERSAMPLE = (int)strtol(argv[++i], NULL, 0); if (OVERSAMPLE<1){ fprintf(stderr,"bad --oversample\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-lpf-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &TX_LPF_BW_HZ)) { fprintf(stderr,"bad --tx-lpf-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--lo")){ NEEDVAL(); if(!parse_hz(argv[++i], &LO_HZ)) { fprintf(stderr,"bad --lo\n"); return 1; } continue; }
        if (!strcmp(a,"--nco")){ NEEDVAL(); if(!parse_hz(argv[++i], &NCO_FREQ_HZ)) { fprintf(stderr,"bad --nco\n"); return 1; } continue; }
        if (!strcmp(a,"--nco-downconvert")){ NEEDVAL(); if(!parse_bool(argv[++i], &NCO_DOWNCONVERT)) { fprintf(stderr,"bad --nco-downconvert\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB = (int)strtol(argv[++i], NULL, 0); if (TX_GAIN_DB<0 || TX_GAIN_DB>73){ fprintf(stderr,"--tx-gain (must be 0..73 dB typical)\n"); } continue; }
        if (!strcmp(a,"--cal-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &CAL_BW_HZ)) { fprintf(stderr,"bad --cal-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--calibrate")){ NEEDVAL(); if(!parse_bool(argv[++i], &DO_CAL)) { fprintf(stderr,"bad --calibrate\n"); return 1; } continue; }

        fprintf(stderr,"unknown option: %s\n", a);
        return 1;
    }
    if (CAL_BW_HZ <= 0) CAL_BW_HZ = TX_LPF_BW_HZ;

    lms_device_t *dev = NULL;
    lms_stream_t txs; memset(&txs,0,sizeof(txs));
    int16_t *buf = NULL;

    signal(SIGINT, on_sigint);

    // Open device
    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) { fprintf(stderr, "no LimeSDR found\n"); return 1; }
    if (LMS_Open(&dev, list[0], NULL)) {
        fprintf(stderr, "LMS_Open failed: %s\n", LMS_GetLastErrorMessage()); return 1;
    }

    // Initialize LMS
    CHECK(LMS_Init(dev));

    // If calibration is requested, reset to defaults *before* calibrating (and before printing BEFORE snapshot).
    if (DO_CAL) {
        CHECK(LMS_Reset(dev));
        printf("device reset to defaults (pre-calibration)\n");
    }

    // Enable TX channel and apply requested settings (this is needed both for pre-cal snapshot and for no-cal mode).
    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    printf("TX channel enabled\n");

    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    print_sr(dev);

    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));

    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    print_gain(dev);

    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));
    print_lo(dev);

    // NCO set (DC -> RF sine)
    {
        double freqs[16] = {0};
        freqs[NCO_INDEX] = NCO_FREQ_HZ;
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
        CHECK(LMS_SetNCOIndex(dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
        print_nco(dev);
    }

    // Snapshot(s)
    if (DO_CAL) {
        print_snapshot(dev, "BEFORE calibration", TX_LPF_BW_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);

        CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, CAL_BW_HZ, 0));  // TX ONLY
        printf("TX calibrated (bw=%.2f MHz)\n", CAL_BW_HZ / 1e6);

        // Some devices may tweak internal paths during calibration; print AFTER state.
        print_snapshot(dev, "AFTER  calibration", TX_LPF_BW_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);
    } else {
        print_snapshot(dev, "Parameters (calibration OFF)", TX_LPF_BW_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);
    }

    // TX stream
    txs.channel = CH;
    txs.isTx = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16)\n", FIFO_SIZE_SAMPLES);

    // Constant DC IQ -> NCO converts to RF sine at LO ± NCO
    buf = (int16_t *)malloc(2 * BUF_SAMPLES * sizeof(int16_t));
    if (!buf) { fprintf(stderr, "malloc failed\n"); goto cleanup; }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0);
    const int16_t Q = 0;
    for (size_t i = 0; i < BUF_SAMPLES; i++) { buf[2*i+0] = I; buf[2*i+1] = Q; }

    double host_sr = 0, rf_sr = 0;
    LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    double lo_now = 0; LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &lo_now);

    const double rf_hz = NCO_DOWNCONVERT ? (lo_now - NCO_FREQ_HZ) : (lo_now + NCO_FREQ_HZ);
    unsigned int g_cur = 0; LMS_GetGaindB(dev, LMS_CH_TX, CH, &g_cur);

    printf("TX %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%u dB, %sconvert)\n",
           rf_hz / 1e6, host_sr / 1e6, rf_sr / 1e6, g_cur, NCO_DOWNCONVERT ? "down" : "up");
    printf("Ctrl+C to stop\n");

    while (keep_running) {
        lms_stream_meta_t meta; memset(&meta, 0, sizeof(meta));
        if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
            fprintf(stderr, "LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }
    }

    printf("\nSIGINT detected\n");

cleanup:
    if (txs.handle) {
        int16_t *z = (int16_t *)calloc(2 * BUF_SAMPLES, sizeof(int16_t));
        if (z) {
            lms_stream_meta_t meta; memset(&meta, 0, sizeof(meta));
            (void)LMS_SendStream(&txs, z, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS);
            free(z);
        }
        LMS_StopStream(&txs);
        LMS_DestroyStream(dev, &txs);
        printf("TX stream stopped\n");
    }

    if (dev) {
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled\n");
        LMS_Close(dev);
    }

    if (buf) free(buf);
    return 0;
}
