#include "lime/LimeSuite.h"
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <time.h>

#define CH 0
#define NCO_INDEX 0
#define FIFO_SIZE_SAMPLES (1 << 17)
#define BUF_SAMPLES 8192
#define SEND_TIMEOUT_MS 1000

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
// clang-format on

static int clampi(int v, int lo, int hi) {
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static int set_mac_channel(lms_device_t *dev, int ch) {
    uint16_t v = 0;
    if (LMS_ReadLMSReg(dev, 0x0020, &v))
        return -1;
    v = (uint16_t)((v & ~0x3) | (ch == 0 ? 0x1 : 0x2));
    return LMS_WriteLMSReg(dev, 0x0020, v);
}

static void print_tx_correctors(lms_device_t *dev, int ch) {
    if (set_mac_channel(dev, ch)) {
        fprintf(stderr, "WARN: can't set MAC for channel %c\n", ch ? 'B' : 'A');
        return;
    }

    uint16_t reg_gq = 0, reg_gi = 0, reg_iq = 0, reg_dc = 0;

    (void)LMS_ReadLMSReg(dev, 0x0201, &reg_gq); // GCORRQ[10:0]
    (void)LMS_ReadLMSReg(dev, 0x0202, &reg_gi); // GCORRI[10:0]
    (void)LMS_ReadLMSReg(dev, 0x0203, &reg_iq); // IQCORR[11:0] (signed)
    (void)LMS_ReadLMSReg(dev, 0x0204, &reg_dc); // DCCORRI[15:8], DCCORRQ[7:0]

    int gq = (int)(reg_gq & 0x07FF);
    int gi = (int)(reg_gi & 0x07FF);

    int iq = (int)(reg_iq & 0x0FFF);
    if (iq & 0x0800)
        iq |= ~0x0FFF;

    int dci = (int8_t)((reg_dc >> 8) & 0xFF);
    int dcq = (int8_t)(reg_dc & 0xFF);

    printf("gain: GCORRI=%d, GCORRQ=%d\n", gi, gq);
    printf("phase: IQCORR=%d\n", iq);
    printf("dc: DCCORRI=%d, DCCORRQ=%d\n", dci, dcq);
}

static int apply_manual_txtsp(lms_device_t *dev, int ch, bool have_gi, int gi, bool have_gq, int gq, bool have_phase,
                              int phase, bool have_dci, int dci, bool have_dcq, int dcq) {
    if (set_mac_channel(dev, ch))
        return -1;

    int rc = 0;

    if (have_gi) {
        gi = clampi(gi, 0, 2047);
        rc |= LMS_WriteLMSReg(dev, 0x0202, (uint16_t)((uint16_t)gi & 0x07FF)); // GCORRI
    }
    if (have_gq) {
        gq = clampi(gq, 0, 2047);
        rc |= LMS_WriteLMSReg(dev, 0x0201, (uint16_t)((uint16_t)gq & 0x07FF)); // GCORRQ
    }
    if (have_phase) {
        phase = clampi(phase, -2047, 2047);
        uint16_t iq12 = (uint16_t)((uint16_t)phase & 0x0FFF); // IQCORR (12 bit signed)
        rc |= LMS_WriteLMSReg(dev, 0x0203, iq12);
    }

    if (have_dci || have_dcq) {
        uint16_t reg_dc = 0;
        rc |= LMS_ReadLMSReg(dev, 0x0204, &reg_dc); // DCCORR
        if (have_dci) {
            int8_t dci_s = (int8_t)clampi(dci, -128, 127);
            reg_dc = (uint16_t)((reg_dc & 0x00FFu) | ((uint16_t)(uint8_t)dci_s << 8)); // I in [15:8]
        }
        if (have_dcq) {
            int8_t dcq_s = (int8_t)clampi(dcq, -128, 127);
            reg_dc = (uint16_t)((reg_dc & 0xFF00u) | (uint16_t)(uint8_t)dcq_s); // Q in [7:0]
        }
        rc |= LMS_WriteLMSReg(dev, 0x0204, reg_dc);
    }

    // Clear PH, GC, DC bypass so manual values take effect
    uint16_t byp = 0;
    rc |= LMS_ReadLMSReg(dev, 0x0208, &byp);
    byp &= (uint16_t)~((1u << 0) | (1u << 1) | (1u << 3)); // clear PH_BYP (bit0), GC_BYP (bit1), DC_BYP (bit3)
    rc |= LMS_WriteLMSReg(dev, 0x0208, byp);

    return rc;
}

int main(int argc, char **argv) {
    int OVERSAMPLE = 32;
    double TX_LPF_BW_HZ = 30e6;
    double LO_HZ = 30e6;
    double NCO_FREQ_HZ = 15e6;
    bool NCO_DOWNCONVERT = true;
    int TX_GAIN_DB = 40;
    double CAL_BW_HZ = -1;

    double HOST_SR_HZ = 5e6;       // MUST be set with --sample-rate
    const char *FIFO_PATH = NULL;  // MUST be set with --fifo
    double SCALE = 1.0;

    bool DO_RESET = false;
    bool DO_CALIBRATE = false;

    bool PRINT_CORRECTORS = false;
    bool SET_GI = false, SET_GQ = false, SET_PHASE = false, SET_DCI = false, SET_DCQ = false;
    int MAN_GI = 0, MAN_GQ = 0, MAN_PHASE = 0, MAN_DCI = 0, MAN_DCQ = 0;

    int fifo_fd = -1;

    // clang-format off
    for (int i=1; i<argc; i++){
        const char* a = argv[i];
        #define NEEDVAL() do{ if (i+1>=argc){ fprintf(stderr,"missing value for %s\n", a); return 1; } }while(0)

        if (!strcmp(a,"--fifo")){ NEEDVAL(); FIFO_PATH = argv[++i]; continue; }
        if (!strcmp(a,"--sample-rate")){ NEEDVAL(); if(!parse_hz(argv[++i], &HOST_SR_HZ)) { fprintf(stderr,"bad --sample-rate\n"); return 1; } continue; }
        if (!strcmp(a,"--oversample")){ NEEDVAL(); OVERSAMPLE = (int)strtol(argv[++i], NULL, 0); if (OVERSAMPLE<1){ fprintf(stderr,"bad --oversample\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-lpf-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &TX_LPF_BW_HZ)) { fprintf(stderr,"bad --tx-lpf-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--lo")){ NEEDVAL(); if(!parse_hz(argv[++i], &LO_HZ)) { fprintf(stderr,"bad --lo\n"); return 1; } continue; }
        if (!strcmp(a,"--nco")){ NEEDVAL(); if(!parse_hz(argv[++i], &NCO_FREQ_HZ)) { fprintf(stderr,"bad --nco\n"); return 1; } continue; }
        if (!strcmp(a,"--nco-downconvert")){ NEEDVAL(); if(!parse_bool(argv[++i], &NCO_DOWNCONVERT)) { fprintf(stderr,"bad --nco-downconvert\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB = (int)strtol(argv[++i], NULL, 0); if (TX_GAIN_DB<0 || TX_GAIN_DB>73){ fprintf(stderr,"--tx-gain (must be 0..73 dB typical)\n"); } continue; }
        if (!strcmp(a,"--cal-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &CAL_BW_HZ)) { fprintf(stderr,"bad --cal-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--scale")){ NEEDVAL(); SCALE = strtod(argv[++i], NULL); if (SCALE<0.0 || SCALE>4.0){ fprintf(stderr,"--scale out of range\n"); return 1; } continue; }
        if (!strcmp(a,"--reset")){ DO_RESET = true; if (i+1 < argc){ bool v=false; if (parse_bool(argv[i+1], &v)){ DO_RESET = v; i++; } } continue; }
        if (!strcmp(a,"--calibrate")){ DO_CALIBRATE = true; if (i+1 < argc){ bool v=false; if (parse_bool(argv[i+1], &v)){ DO_CALIBRATE = v; i++; } } continue; }
        if (!strcmp(a,"--print-correctors")){ PRINT_CORRECTORS = true; if (i+1 < argc){ bool v=false; if (parse_bool(argv[i+1], &v)){ PRINT_CORRECTORS = v; i++; } } continue; }
        if (!strcmp(a,"--set-gain-i")){ NEEDVAL(); SET_GI=true; MAN_GI=clampi((int)strtol(argv[++i], NULL, 0), 0, 2047); continue; }
        if (!strcmp(a,"--set-gain-q")){ NEEDVAL(); SET_GQ=true; MAN_GQ=clampi((int)strtol(argv[++i], NULL, 0), 0, 2047); continue; }
        if (!strcmp(a,"--set-phase")) { NEEDVAL(); SET_PHASE=true; MAN_PHASE=clampi((int)strtol(argv[++i], NULL, 0), -2047, 2047); continue; }
        if (!strcmp(a,"--set-dc-i"))  { NEEDVAL(); SET_DCI=true; MAN_DCI=clampi((int)strtol(argv[++i], NULL, 0), -128, 127); continue; }
        if (!strcmp(a,"--set-dc-q"))  { NEEDVAL(); SET_DCQ=true; MAN_DCQ=clampi((int)strtol(argv[++i], NULL, 0), -128, 127); continue; }

        fprintf(stderr,"unknown option: %s\n", a);
        return 1;
    }
    if (!FIFO_PATH){ fprintf(stderr,"missing --fifo <path>\n"); return 1; }
    if (CAL_BW_HZ <= 0) CAL_BW_HZ = TX_LPF_BW_HZ;
    // clang-format on

    signal(SIGINT, on_sigint);

    lms_device_t *dev = NULL;
    lms_stream_t txs;
    int16_t *buf = NULL;
    memset(&txs, 0, sizeof(txs));

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

    if (DO_RESET) {
        CHECK(LMS_Reset(dev));
        printf("device reset to defaults\n");
    }
    CHECK(LMS_Init(dev));

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
        print_nco(dev);
    }

    if (DO_CALIBRATE) {
        CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, CAL_BW_HZ, 0));
        printf("TX calibrated (bw=%.2f MHz)\n", CAL_BW_HZ / 1e6);
    }

    if (PRINT_CORRECTORS) {
        print_tx_correctors(dev, CH);
    }

    if (SET_GI || SET_GQ || SET_PHASE || SET_DCI || SET_DCQ) {
        CHECK(apply_manual_txtsp(dev, CH, SET_GI, MAN_GI, SET_GQ, MAN_GQ, SET_PHASE, MAN_PHASE, SET_DCI, MAN_DCI,
                                 SET_DCQ, MAN_DCQ));
        printf("Manual TXTSP correctors applied.\n");
        if (PRINT_CORRECTORS) {
            print_tx_correctors(dev, CH);
        }
    }

    txs.channel = CH;
    txs.isTx = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16)\n", FIFO_SIZE_SAMPLES);

    double host_sr = 0, rf_sr = 0;
    LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    unsigned int g_cur = 0;
    LMS_GetGaindB(dev, LMS_CH_TX, CH, &g_cur);

    const double rf_hz = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
    printf("TX %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%u dB, %sconvert)\n",
           rf_hz / 1e6, host_sr / 1e6, rf_sr / 1e6, g_cur, NCO_DOWNCONVERT ? "down" : "up");

    printf("Opening FIFO %s for reading (blocking until writer connects)...\n", FIFO_PATH);
    fifo_fd = open(FIFO_PATH, O_RDONLY);
    if (fifo_fd < 0) {
        perror("open fifo");
        goto cleanup;
    }
    printf("FIFO opened, streaming IQ from FIFO (Ctrl+C to stop)\n");

    buf = (int16_t *)malloc(2 * BUF_SAMPLES * sizeof(int16_t));
    if (!buf) {
        fprintf(stderr, "malloc failed\n");
        goto cleanup;
    }

    const size_t bytes_per_frame = 2 * sizeof(int16_t); // I + Q, 16-bit each
    const size_t bytes_per_chunk = BUF_SAMPLES * bytes_per_frame;

    lms_stream_status_t st;
    memset(&st, 0, sizeof(st));

    time_t last = time(NULL);

    while (keep_running) {
        struct timespec t1, t2;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        ssize_t got = read(fifo_fd, buf, bytes_per_chunk);
        clock_gettime(CLOCK_MONOTONIC, &t2);

        if (got < 0) {
            if (errno == EINTR)
                continue;
            perror("read fifo");
            break;
        }
        if (got == 0) {
            fprintf(stderr, "FIFO EOF (writer closed), stopping\n");
            break;
        }

        double dt = (t2.tv_sec - t1.tv_sec) +
                    (t2.tv_nsec - t1.tv_nsec)/1e9;
        if (dt > 0.01) {
            fprintf(stderr, "read from fifo blocked %.3f s\n", dt);
        }

        if (SCALE != 1.0) {
            size_t samples16 = (size_t)got / 2;
            for (size_t i = 0; i < samples16; i++) {
                int32_t v = (int32_t)buf[i];
                int32_t s = (int32_t)(v * SCALE);
                if (s > 32767)
                    s = 32767;
                else if (s < -32768)
                    s = -32768;
                buf[i] = (int16_t)s;
            }
        }

        size_t frames = (size_t)got / bytes_per_frame;
        if (frames > 0) {
            lms_stream_meta_t meta;
            memset(&meta, 0, sizeof(meta));
            if (LMS_SendStream(&txs, buf, (int)frames, &meta, SEND_TIMEOUT_MS) < 0) {
                fprintf(stderr, "LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
                break;
            }
        }

        time_t now = time(NULL);
        if (now != last) {
            last = now;
            if (!LMS_GetStreamStatus(&txs, &st)) {
                printf("TX status: fifo=%u, underrun=%u, overrun=%u\n",
                       st.fifoFilledCount, st.underrun, st.overrun);
            }
        }
    }

    printf("\nSIGINT or FIFO EOF, stopping\n");

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

    if (fifo_fd >= 0)
        close(fifo_fd);

    if (dev) {
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled\n");
        LMS_Close(dev);
    }

    if (buf)
        free(buf);
    return 0;
}
