#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <signal.h>
#include <math.h>

#define CH                0          // TX channel A
#define HOST_SR_HZ        5000000    // 5 Msps over USB
#define OVERSAMPLE        8          // RF ≈ HOST_SR * OVERSAMPLE = 40 Msps
#define TX_LPF_BW_HZ      50000000   // 50 MHz LPF
#define LO_HZ             30000000   // 30 MHz LO (PLL-friendly)
#define TX_GAIN_DB        40         // moderate TX gain
#define FIFO_SIZE_SAMPLES (1<<17)    // bigger FIFO for stability
#define BUF_SAMPLES       8192       // larger chunk reduces IRQ/USB churn
#define SEND_TIMEOUT_MS   1000
#define TONE_SCALE        0.70       // 70% FS to avoid DAC clipping

// Sweep parameters
#define START_MHZ         10
#define END_MHZ           30
#define STEP_MHZ          1
#define DWELL_SECONDS     2.0

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

    // LO fixed at 30 MHz
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));

    // 3) Preload NCO bank (we’ll keep using index 0, downconvert mode)
    {
        double freqs[16]={0};
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));  // dir_tx=true
        CHECK(LMS_SetNCOIndex    (dev, true, CH, 0 /*index*/, true /*downconvert*/));
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

    printf("Sweeping TX from %d to %d MHz (step %d MHz), dwell ~%.1f s "
           "(host=%.2f Msps, rf=%.2f Msps, gain=%u dB). Ctrl+C to stop.\n",
           START_MHZ, END_MHZ, STEP_MHZ, DWELL_SECONDS, host_sr/1e6, rf_sr/1e6, g);

    // Helper: how many buffer sends ≈ 2 seconds at host_sr
    const double sec_per_chunk = (double)BUF_SAMPLES / host_sr;
    const int sends_per_dwell  = (int)ceil(DWELL_SECONDS / sec_per_chunk);

    // 5) Sweep loop (reprogram NCO for desired RF = LO - NCO)
    while (keep_running) {
        for (int rf_mhz = START_MHZ; rf_mhz <= END_MHZ && keep_running; rf_mhz += STEP_MHZ) {

            const double desired_rf_hz = (double)rf_mhz * 1e6;
            double nco_hz = LO_HZ - desired_rf_hz;          // downconvert: RF = LO - NCO
            if (nco_hz < 0) nco_hz = 0;                     // clamp (shouldn’t happen in this range)
            const double nco_max = rf_sr / 2.0;             // NCO limit ~ rf sample rate / 2
            if (nco_hz > nco_max) {
                fprintf(stderr, "Skip %d MHz: required NCO=%.1f MHz exceeds limit %.1f MHz\n",
                        rf_mhz, nco_hz/1e6, nco_max/1e6);
                continue;
            }

            // Program NCO index 0 to nco_hz, downconvert true
            double freqs[16]={0};
            freqs[0] = nco_hz;
            CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
            CHECK(LMS_SetNCOIndex    (dev, true, CH, 0, true));

            printf("TX @ %d MHz (NCO=%.3f MHz)\n", rf_mhz, nco_hz/1e6);

            // Transmit for ~DWELL_SECONDS at this frequency
            for (int k=0; k<sends_per_dwell && keep_running; ++k) {
                lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
                if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0) {
                    fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
                    keep_running = 0;
                    break;
                }
            }
        }
        // Optional: loop forever. Comment the next line to keep sweeping;
        // otherwise stop after one pass:
        // break;
    }

cleanup:
    if (buf) free(buf);
    if (txs.handle) { LMS_StopStream(&txs); LMS_DestroyStream(dev, &txs); }
    if (dev) LMS_Close(dev);
    return 0;
}
