#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>
#include <stdbool.h>

#define TX_CH               0
#define RX_CH               0

#define HOST_SR_HZ          5000000      // 5 Msps
#define OVERSAMPLE          8            // ≈40 Msps RF-side
#define TX_LPF_BW_HZ        50000000     // 50 MHz TX LPF (your original)
#define RX_LPF_BW_HZ        10000000     // 10 MHz RX LPF to help cal
#define TX_GAIN_DB          40

#define FIFO_SIZE_SAMPLES   (1<<17)
#define BUF_SAMPLES         8192
#define SEND_TIMEOUT_MS     1000
#define TONE_SCALE          0.70

// Target RF
#define RF_TARGET_HZ        15000000     // 15 MHz
#define NCO_MAX_HZ          20000000     // NCO limit ~ Fs/2 (here 20 MHz)

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

static int do_cal(double rx_bw_hz, double tx_bw_hz)
{
    int ok = 1;
    if (LMS_Calibrate(dev, LMS_CH_RX, RX_CH, rx_bw_hz, 0) != 0) {
        fprintf(stderr, "WARN: RX Calibrate failed: %s — continuing to try.\n",
                LMS_GetLastErrorMessage());
        ok = 0;
    }
    if (LMS_Calibrate(dev, LMS_CH_TX, TX_CH, tx_bw_hz, 0) != 0) {
        fprintf(stderr, "WARN: TX Calibrate failed: %s — continuing to try.\n",
                LMS_GetLastErrorMessage());
        ok = 0;
    }
    return ok;
}

static int tune_lo_and_nco(double lo_hz, double *nco_hz_out)
{
    double nco = lo_hz - RF_TARGET_HZ;   // downconvert → RF = LO - NCO
    if (nco < 0) nco = -nco;             // safeguard (we’ll always use downconvert)
    if (nco > NCO_MAX_HZ) {
        fprintf(stderr, "Skip LO %.3f MHz: NCO %.3f MHz exceeds %.3f MHz\n",
                lo_hz/1e6, nco/1e6, NCO_MAX_HZ/1e6);
        return 0;
    }

    // Set LO for both TX and RX (needed for loopback-based cal)
    if (LMS_SetLOFrequency(dev, LMS_CH_TX, TX_CH, lo_hz) != 0) return 0;
    if (LMS_SetLOFrequency(dev, LMS_CH_RX, RX_CH, lo_hz) != 0) return 0;

    // Program NCO for TX path (downconvert)
    double freqs[16] = {0};
    freqs[0] = nco;
    if (LMS_SetNCOFrequency(dev, true, TX_CH, freqs, 0.0) != 0) return 0;
    if (LMS_SetNCOIndex    (dev, true, TX_CH, 0, /*downconvert=*/true) != 0) return 0;

    *nco_hz_out = nco;
    return 1;
}

int main(void)
{
    memset(&txs, 0, sizeof(txs));
    signal(SIGINT, on_sigint);

    // --- Open ---
    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) { fprintf(stderr,"No LimeSDR found\n"); return 1; }
    CHECK(LMS_Open(&dev, list[0], NULL));
    CHECK(LMS_Init(dev));

    // --- Enable paths ---
    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, TX_CH, true));
    CHECK(LMS_EnableChannel(dev, LMS_CH_RX, RX_CH, true));

    // Antenna paths (adjust for your board if different)
    // LimeSDR-USB typical:
    //   TX: TX1_1 (or TX2_1)        RX: LNAW (wideband) or LNAL/LNAH as needed
    LMS_SetAntenna(dev, LMS_CH_TX, TX_CH, LMS_PATH_TX1);
    LMS_SetAntenna(dev, LMS_CH_RX, RX_CH, LMS_PATH_LNAW);

    // --- Rates & filters ---
    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    double host_sr=0, rf_sr=0; CHECK(LMS_GetSampleRate(dev, LMS_CH_TX, TX_CH, &host_sr, &rf_sr));
    CHECK(LMS_SetLPFBW(dev, LMS_CH_RX, RX_CH, RX_LPF_BW_HZ));
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, TX_CH, TX_LPF_BW_HZ));
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, TX_CH, TX_GAIN_DB));
    unsigned int g=0; CHECK(LMS_GetGaindB(dev, LMS_CH_TX, TX_CH, &g));

    // --- Try a few LO/NCO candidates to get reliable SXR lock ---
    // Keep LO not too close to the 30 MHz floor; keep NCO <= 20 MHz.
    const double lo_candidates[] = {
        34e6, // NCO=19 → good margin below 20
        35e6, // NCO=20 → at the common limit, often still OK
        33e6  // NCO=18 → fallback (a bit closer to floor but often fine)
    };

    double chosen_lo = 0.0, chosen_nco = 0.0;
    int calibrated = 0;
    for (size_t i=0;i<sizeof(lo_candidates)/sizeof(lo_candidates[0]);++i) {
        double lo = lo_candidates[i], nco;
        if (!tune_lo_and_nco(lo, &nco)) {
            fprintf(stderr, "LO/NCO setup failed at LO=%.3f MHz\n", lo/1e6);
            continue;
        }
        // Calibrate with modest BW to help PLLs converge
        if (do_cal(8e6, 8e6)) {
            chosen_lo = lo; chosen_nco = nco; calibrated = 1;
            break;
        }
    }
    if (!calibrated) {
        fprintf(stderr, "WARN: Calibration did not fully complete — proceeding anyway.\n");
        // Use the last successful tune (or first candidate if none reported error)
        chosen_lo = (chosen_lo>0)? chosen_lo : lo_candidates[0];
        (void)tune_lo_and_nco(chosen_lo, &chosen_nco);
    }

    // --- TX stream ---
    txs.channel  = TX_CH;
    txs.isTx     = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt  = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));

    // --- Constant tone (I-only) ---
    int16_t* buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf) { fprintf(stderr,"malloc failed\n"); cleanup_and_exit(1); }
    const int16_t I = (int16_t)(TONE_SCALE * 32767.0), Q = 0;
    for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i]=I; buf[2*i+1]=Q; }

    printf("TX ~%.1f MHz (host=%.2f Msps rf=%.2f Msps gain=%u dB) LO=%.3f MHz NCO=%.3f MHz. Ctrl+C to stop.\n",
           RF_TARGET_HZ/1e6, host_sr/1e6, rf_sr/1e6, g, chosen_lo/1e6, chosen_nco/1e6);

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