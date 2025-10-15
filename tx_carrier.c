#include "lime/LimeSuite.h"
#include <ctype.h>
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CH 0
#define NCO_INDEX 0
#define FIFO_SIZE_SAMPLES (1 << 17)
#define BUF_SAMPLES 8192
#define SEND_TIMEOUT_MS 1000
#define TONE_SCALE 0.70

// clang-format off
#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)
// clang-format on

static volatile int keep_running = 1;
static void on_sigint(int s) {
    (void)s;
    keep_running = 0;
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

// clang-format off
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

int main(int argc, char** argv)
{
    double HOST_SR_HZ      = 5e6;
    int    OVERSAMPLE      = 32;
    double TX_LPF_BW_HZ    = 20e6;
    double LO_HZ           = 30e6;
    double NCO_FREQ_HZ     = 15e6;
    bool   NCO_DOWNCONVERT = true;
    int    TX_GAIN_DB      = 40;
    double CAL_BW_HZ       = -1;

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

        fprintf(stderr,"unknown option: %s\n", a);
        return 1;
    }
    if (CAL_BW_HZ <= 0) CAL_BW_HZ = TX_LPF_BW_HZ;
    // clang-format on

    lms_device_t *dev = NULL;
    lms_stream_t txs;
    int16_t *buf = NULL;

    signal(SIGINT, on_sigint);

    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) {
        fprintf(stderr, "no LimeSDR found\n");
        return 1;
    }
    if (LMS_Open(&dev, list[0], NULL)) {
        fprintf(stderr, "LMS_Open failed: %s\n", LMS_GetLastErrorMessage());
        return 1;
    }

    CHECK(LMS_Init(dev));

    CHECK(LMS_Reset(dev));
    printf("device reset to defaults\n");

    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    printf("TX channel enabled\n");

    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    print_sr(dev);

    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));

    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    print_gain(dev);

    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));
    print_lo(dev);

    {
        double freqs[16] = {0};
        freqs[NCO_INDEX] = NCO_FREQ_HZ;
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
        CHECK(LMS_SetNCOIndex(dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
        int idx = LMS_GetNCOIndex(dev, true, CH);
        if (idx < 0) {
            fprintf(stderr, "LMS_GetNCOIndex failed: %s\n", LMS_GetLastErrorMessage());
            goto cleanup;
        }
        print_nco(dev);
    }

    CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, CAL_BW_HZ, 0));
    printf("TX calibrated (bw=%.2f MHz)\n", CAL_BW_HZ / 1e6);

    memset(&txs, 0, sizeof(txs));
    txs.channel = CH;
    txs.isTx = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16)\n", FIFO_SIZE_SAMPLES);

    buf = (int16_t *)malloc(2 * BUF_SAMPLES * sizeof(int16_t));
    if (!buf) {
        fprintf(stderr, "malloc failed\n");
        goto cleanup;
    }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0);
    const int16_t Q = 0;
    for (size_t i = 0; i < BUF_SAMPLES; i++) {
        buf[2 * i + 0] = I;
        buf[2 * i + 1] = Q;
    }

    double host_sr = 0, rf_sr = 0;
    LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    unsigned int g_cur = 0;
    LMS_GetGaindB(dev, LMS_CH_TX, CH, &g_cur);

    const double rf_hz = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
    printf("TX %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%u dB, %sconvert)\n", rf_hz / 1e6, host_sr / 1e6,
           rf_sr / 1e6, g_cur, NCO_DOWNCONVERT ? "down" : "up");
    printf("Ctrl+C to stop\n");

    while (keep_running) {
        lms_stream_meta_t meta;
        memset(&meta, 0, sizeof(meta));
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
            lms_stream_meta_t meta;
            memset(&meta, 0, sizeof(meta));
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

    if (buf)
        free(buf);
    return 0;
}
