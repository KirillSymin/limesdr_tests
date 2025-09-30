#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <signal.h>

#define CH                0
#define HOST_SR_HZ        5000000
#define OVERSAMPLE        8               // RF ≈ 40 Msps (NCO range up to 20 MHz)
#define TX_LPF_BW_HZ      50000000
#define LO_HZ             30000000
#define NCO_FREQ_HZ       15000000
#define NCO_INDEX         0
#define NCO_DOWNCONVERT   true
#define TX_GAIN_DB        40
#define FIFO_SIZE_SAMPLES (1<<17)
#define BUF_SAMPLES       8192
#define SEND_TIMEOUT_MS   1000
#define TONE_SCALE        0.70

static lms_device_t* dev = NULL;
static lms_stream_t  txs; // zeroed in main
static volatile int keep_running = 1;

static void on_sigint(int s){ (void)s; keep_running = 0; }

static void cleanup_and_exit(int code)
{
    if (txs.handle) { LMS_StopStream(&txs); LMS_DestroyStream(dev, &txs); }
    if (dev) LMS_Close(dev);
    exit(code);
}

#define CHECK(call) do { \
    int __e = (call); \
    if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #call, LMS_GetLastErrorMessage()); cleanup_and_exit(1); } \
} while(0)

int main(void)
{
    memset(&txs, 0, sizeof(txs));
    signal(SIGINT, on_sigint);

    // Open device
    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) { fprintf(stderr,"No LimeSDR found\n"); return 1; }
    CHECK(LMS_Open(&dev, list[0], NULL));
    CHECK(LMS_Init(dev));
    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));

    // Rates: host 5 Msps, RF ≈ 40 Msps
    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    double host_sr=0, rf_sr=0; CHECK(LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr));

    // LPF & Gain
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    unsigned int g=0; CHECK(LMS_GetGaindB(dev, LMS_CH_TX, CH, &g));

    // LO
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));

    // Try calibration (smaller BW). If it fails, warn and continue.
    if (LMS_Calibrate(dev, LMS_CH_TX, CH, 8000000, 0) != 0) {
        fprintf(stderr, "WARN: TX Calibrate failed: %s — continuing without cal.\n",
                LMS_GetLastErrorMessage());
    }

    // NCO: 15 MHz downconvert
    double freqs[16] = {0};
    freqs[NCO_INDEX] = (double)NCO_FREQ_HZ; // positive; direction set by downconvert flag
    CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
    CHECK(LMS_SetNCOIndex    (dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));

    // Stream
    txs.channel = CH; txs.isTx = true; txs.fifoSize = FIFO_SIZE_SAMPLES; txs.dataFmt = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));

    int16_t* buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf) { fprintf(stderr,"malloc failed\n"); cleanup_and_exit(1); }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0), Q = 0;
    for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i]=I; buf[2*i+1]=Q; }

    printf("TX ~%.1f MHz (host=%.2f Msps rf=%.2f Msps gain=%u dB). Ctrl+C to stop.\n",
           (LO_HZ - NCO_FREQ_HZ)/1e6, host_sr/1e6, rf_sr/1e6, g);

    while (keep_running) {
        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }
    }

    free(buf);
    cleanup_and_exit(0);
    return 0;
}