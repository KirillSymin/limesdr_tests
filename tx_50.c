#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <signal.h>

#define CH                0          // TX channel A
#define HOST_SR_HZ        5000000    // 5 Msps over USB
#define OVERSAMPLE        8          // RF ≈ HOST_SR * OVERSAMPLE = 40 Msps
#define TX_LPF_BW_HZ      50000000   // 50 MHz LPF
#define LO_HZ             30000000   // 30 MHz LO (PLL-friendly)
#define NCO_FREQ_HZ       15000000   // 15 MHz NCO magnitude
#define NCO_INDEX         0
#define NCO_DOWNCONVERT   true       // RF = LO - NCO = 15 MHz
#define TX_GAIN_DB        40         // moderate TX gain
#define FIFO_SIZE_SAMPLES (1<<17)    // bigger FIFO for stability
#define BUF_SAMPLES       8192       // larger chunk reduces IRQ/USB churn
#define SEND_TIMEOUT_MS   1000
#define TONE_SCALE        0.70       // 70% FS to avoid DAC clipping

#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)

static volatile int keep_running = 1;
static void on_sigint(int s){ (void)s; keep_running = 0; }

int main(void)
{
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

    // Sample rates: host 5 Msps, RF ≈ 40 Msps (for NCO range up to 20 MHz)
    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    double host_sr=0, rf_sr=0;
    CHECK(LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr));

    // TX LPF/BW
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));

    // Gain
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    unsigned int g=0; CHECK(LMS_GetGaindB(dev, LMS_CH_TX, CH, &g));

    // LO
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));

    // Calibrate near operating BW
    CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, 20000000, 0)); // 20 MHz calib BW

    // 3) NCO: program 15 MHz and select downconvert so RF = 30 - 15 = 15 MHz
    {
        double freqs[16]={0};
        freqs[NCO_INDEX] = (double)NCO_FREQ_HZ; // positive; direction set below
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));              // dir_tx=true
        CHECK(LMS_SetNCOIndex    (dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
        int idx = LMS_GetNCOIndex(dev, true, CH);
        if (idx < 0) { fprintf(stderr,"LMS_GetNCOIndex failed: %s\n", LMS_GetLastErrorMessage()); goto cleanup; }
    }

    // 4) TX stream
    memset(&txs, 0, sizeof(txs));
    txs.channel  = CH;
    txs.isTx     = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt  = LMS_FMT_I16; // C-safe
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));

    // Constant DC IQ -> pure RF tone after NCO
    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf) { fprintf(stderr,"malloc failed\n"); goto cleanup; }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0);
    const int16_t Q = 0;
    for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=I; buf[2*i+1]=Q; }

    printf("TX @ %.1f MHz  (host=%.2f Msps, rf=%.2f Msps, gain=%u dB). Ctrl+C to stop.\n",
           (LO_HZ - NCO_FREQ_HZ)/1e6, host_sr/1e6, rf_sr/1e6, g);

    // 5) Stream loop
    while (keep_running) {
        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }
    }

cleanup:
    if (buf) free(buf);
    if (txs.handle) { LMS_StopStream(&txs); LMS_DestroyStream(dev, &txs); }
    if (dev) LMS_Close(dev);
    return 0;
}