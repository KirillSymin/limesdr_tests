#include "lime/LimeSuite.h"
#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CH 0
#define FIFO_SIZE_SAMPLES (1 << 17)
#define BUF_SAMPLES 8192
#define SEND_TIMEOUT_MS 1000
#define AMP_SCALE 0.65

// clang-format off
#define CHECK(x) do { int __e = (x); if (__e) { fprintf(stderr, "ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } } while (0)
// clang-format on

static volatile int keep_running = 1;
static void on_sigint(int s) {
    (void)s;
    keep_running = 0;
}

// clang-format off
static bool parse_bool(const char *s, bool *out) {
    if (!s || !out) return false;
    if (!strcasecmp(s, "1") || !strcasecmp(s, "true") || !strcasecmp(s, "yes") || !strcasecmp(s, "on")) {
        *out = true;
        return true;
    }
    if (!strcasecmp(s, "0") || !strcasecmp(s, "false") || !strcasecmp(s, "no") || !strcasecmp(s, "off")) {
        *out = false;
        return true;
    }
    return false;
}

static bool parse_hz(const char *s, double *out) {
    if (!s || !out) return false;
    char *end = NULL;
    double v = strtod(s, &end);
    if (end && *end != '\0') {
        while (*end && isspace((unsigned char)*end)) end++;
        if (*end) {
            double mul = 1.0;
            char c = (char)tolower((unsigned char)*end);
            if (c == 'k') mul = 1e3;
            else if (c == 'm') mul = 1e6;
            else if (c == 'g') mul = 1e9;
            else return false;
            end++;
            while (*end && isspace((unsigned char)*end)) end++;
            if (*end != '\0') return false;
            v *= mul;
        }
    }
    *out = v;
    return true;
}

int main(int argc, char **argv) {
    double HOST_SR_HZ = 5e6;
    double TX_LPF_BW_HZ = 20e6;
    double LO_HZ = 30e6;
    double TONE_HZ = 1e6;
    bool USB = true;
    int TX_GAIN_DB = 40;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        #define NEEDVAL() do { if (i+1 >= argc) { fprintf(stderr, "missing value for %s\n", a); return 1; } } while (0)
        if (!strcmp(a, "--host-sr")) { NEEDVAL(); if (!parse_hz(argv[++i], &HOST_SR_HZ)) { fprintf(stderr, "bad --host-sr\n"); return 1; } continue; }
        if (!strcmp(a, "--tx-lpf-bw")) { NEEDVAL(); if (!parse_hz(argv[++i], &TX_LPF_BW_HZ)) { fprintf(stderr, "bad --tx-lpf-bw\n"); return 1; } continue; }
        if (!strcmp(a, "--lo")) { NEEDVAL(); if (!parse_hz(argv[++i], &LO_HZ)) { fprintf(stderr, "bad --lo\n"); return 1; } continue; }
        if (!strcmp(a, "--tone")) { NEEDVAL(); if (!parse_hz(argv[++i], &TONE_HZ)) { fprintf(stderr, "bad --tone\n"); return 1; } continue; }
        if (!strcmp(a, "--usb")) { NEEDVAL(); if (!parse_bool(argv[++i], &USB)) { fprintf(stderr, "bad --usb\n"); return 1; } continue; }
        if (!strcmp(a, "--tx-gain")) { NEEDVAL(); TX_GAIN_DB = (int)strtol(argv[++i], NULL, 0); if (TX_GAIN_DB < 0 || TX_GAIN_DB > 73) { fprintf(stderr, "--tx-gain out of range\n"); return 1; } continue; }
        fprintf(stderr, "unknown option: %s\n", a);
        return 1;
    }

    if (TONE_HZ <= 0 || TONE_HZ >= HOST_SR_HZ / 2) {
        fprintf(stderr, "tone must be in (0, host_sr/2)\n");
        return 1;
    }
    // clang-format on

    signal(SIGINT, on_sigint);

    lms_device_t *dev = NULL;
    lms_stream_t txs;
    int16_t *buf = NULL;

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

    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, 0));
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));
    CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ, 0));

    memset(&txs, 0, sizeof(txs));
    txs.channel = CH;
    txs.isTx = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));

    buf = (int16_t *)malloc(2 * BUF_SAMPLES * sizeof(int16_t));
    if (!buf) {
        fprintf(stderr, "malloc failed\n");
        goto cleanup;
    }

    const double two_pi = 2.0 * M_PI;
    const double ph_inc = two_pi * (TONE_HZ / HOST_SR_HZ);
    double phase = 0.0;
    const double scale = AMP_SCALE * 32767.0;
    const double q_sign = USB ? +1.0 : -1.0;

    double host_sr = 0, rf_sr = 0;
    LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    unsigned g_cur = 0;
    LMS_GetGaindB(dev, LMS_CH_TX, CH, &g_cur);
    const double rf_hz = USB ? (LO_HZ + TONE_HZ) : (LO_HZ - TONE_HZ);
    printf("TX SSB tone at %.6f MHz (LO=%.6f, tone=%.3f kHz, %s)\n", rf_hz / 1e6, LO_HZ / 1e6, TONE_HZ / 1e3,
           USB ? "USB" : "LSB");
    printf("Sample rate host=%.2f Msps, rf=%.2f Msps, gain=%u dB\n", host_sr / 1e6, rf_sr / 1e6, g_cur);
    printf("Ctrl+C to stop\n");

    while (keep_running) {
        for (int i = 0; i < BUF_SAMPLES; i++) {
            float c = (float)cos(phase);
            float s_ = (float)sin(phase);
            int16_t I = (int16_t)(scale * c);
            int16_t Q = (int16_t)(scale * (q_sign * s_));
            buf[2 * i + 0] = I;
            buf[2 * i + 1] = Q;
            phase += ph_inc;
            if (phase >= two_pi)
                phase -= two_pi;
        }

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
        LMS_Close(dev);
    }
    if (buf)
        free(buf);
    return 0;
}
