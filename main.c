#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <stdbool.h>

/* -------- tunable constants -------- */
#define CFG_CH               0
#define CFG_HOST_SR          5e6
#define CFG_LPF_BW           30e6
#define CFG_TX_ANT           0
#define CFG_TX_GAIN_DB       40
#define CFG_LO_FREQ          30e6
#define CFG_CAL_BW           20e6
#define CFG_NCO_FREQ_HZ      15e6        /* magnitude; sign set via downconvert flag */
#define CFG_NCO_INDEX        0
#define CFG_NCO_DOWNCONVERT  true        /* true => RF = LO - fNCO */
#define CFG_FIFO_SIZE        (1<<16)
#define CFG_TONE_I_SCALE     0.7
#define CFG_TONE_Q           0
#define CFG_BUF_SAMPLES      4096
#define CFG_SEND_TIMEOUT_MS  1000

/* pick the right enum name depending on compiler */
#ifdef __cplusplus
  #define STREAM_FMT  lms_stream_t::LMS_FMT_I16
#else
  #define STREAM_FMT  LMS_FMT_I16
#endif

/* -------------- helpers -------------- */
#define CHECK(call) do { \
    int __e = (call); \
    if (__e != 0) { \
        fprintf(stderr, "ERROR: %s -> %s\n", #call, LMS_GetLastErrorMessage()); \
        cleanup_and_exit(1); \
    } \
} while(0)

static lms_device_t* dev = NULL;
static lms_stream_t  txs;

static void cleanup_and_exit(int code)
{
    if (txs.handle) LMS_DestroyStream(dev, &txs);
    if (dev) { LMS_Close(dev); dev = NULL; }
    exit(code);
}

static void print_stream_status(lms_stream_t* s, const char* tag)
{
    lms_stream_status_t st;
    CHECK(LMS_GetStreamStatus(s, &st));
    printf("[%s] fifo: %u/%u  underrun:%u overrun:%u  dropped:%u  ts: %" PRIu64 "\n",
           tag, st.fifoFilledCount, s->fifoSize, st.underrun, st.overrun, st.droppedPackets, st.timestamp);
}

int main(void)
{
    /* 1) Open device */
    int n;
    lms_info_str_t list[8];
    n = LMS_GetDeviceList(list);
    if (n < 0) { fprintf(stderr, "LMS_GetDeviceList failed: %s\n", LMS_GetLastErrorMessage()); return 1; }
    if (n == 0) { fprintf(stderr, "No LimeSDR found.\n"); return 1; }
    printf("Using device: %s\n", list[0]);
    if (LMS_Open(&dev, list[0], NULL) != 0) {
        fprintf(stderr, "LMS_Open failed: %s\n", LMS_GetLastErrorMessage());
        return 1;
    }

    /* 2) Init and enable TX channel */
    CHECK(LMS_Init(dev));
    const size_t ch = CFG_CH;
    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, ch, true));

    /* 3) Sample rate */
    double host_sr = CFG_HOST_SR, rf_sr = 0.0;
    CHECK(LMS_SetSampleRate(dev, host_sr, 1));
    CHECK(LMS_GetSampleRate(dev, LMS_CH_TX, ch, &host_sr, &rf_sr));
    printf("TX sample rate: host=%.6f Msps  rf=%.6f Msps\n", host_sr/1e6, rf_sr/1e6);

    /* 4) LPF BW */
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, ch, CFG_LPF_BW));
    double bw = 0.0;
    CHECK(LMS_GetLPFBW(dev, LMS_CH_TX, ch, &bw));
    printf("TX LPF BW: %.3f MHz\n", bw/1e6);

    /* 5) Antenna & gain */
    CHECK(LMS_SetAntenna(dev, LMS_CH_TX, ch, CFG_TX_ANT));
    int ant = LMS_GetAntenna(dev, LMS_CH_TX, ch);
    if (ant < 0) { fprintf(stderr, "LMS_GetAntenna failed: %s\n", LMS_GetLastErrorMessage()); cleanup_and_exit(1); }
    printf("TX antenna index: %d\n", ant);

    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, ch, CFG_TX_GAIN_DB));
    /* this header expects unsigned int* */
    unsigned int gain_db = 0;
    CHECK(LMS_GetGaindB(dev, LMS_CH_TX, ch, &gain_db));
    printf("TX gain: %u dB\n", gain_db);

    /* 6) LO freq */
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, ch, CFG_LO_FREQ));
    double lo_rd = 0.0;
    CHECK(LMS_GetLOFrequency(dev, LMS_CH_TX, ch, &lo_rd));
    printf("TX LO: %.6f MHz\n", lo_rd/1e6);

    /* 7) Calibrate */
    CHECK(LMS_Calibrate(dev, LMS_CH_TX, ch, CFG_CAL_BW, 0));
    printf("TX Calibrate done\n");

    /* 8) Program NCO table (float_type == double in your headers) */
    {
        double nco_freqs[16] = {0};
        nco_freqs[CFG_NCO_INDEX] = fabs((double)CFG_NCO_FREQ_HZ);
        /* LMS_SetNCOFrequency(dev, dir_tx, chan, freq[], pho) */
        CHECK(LMS_SetNCOFrequency(dev, /*dir_tx=*/true, ch, nco_freqs, 0.0));

        /* LMS_SetNCOIndex(dev, dir_tx, chan, index, downconvert) */
        CHECK(LMS_SetNCOIndex(dev, /*dir_tx=*/true, ch, CFG_NCO_INDEX, CFG_NCO_DOWNCONVERT));

        /* Your LMS_GetNCOIndex takes only 3 args and RETURNS the index */
        int nco_idx = LMS_GetNCOIndex(dev, /*dir_tx=*/true, ch);
        if (nco_idx < 0) {
            fprintf(stderr, "LMS_GetNCOIndex failed: %s\n", LMS_GetLastErrorMessage());
            cleanup_and_exit(1);
        }
        printf("TX NCO index: %d (direction set when index was programmed)\n", nco_idx);

        /* Read back frequency table; phase readback is optional */
        double nco_read[16] = {0}, pho_read = 0.0;
        CHECK(LMS_GetNCOFrequency(dev, /*dir_tx=*/true, ch, nco_read, &pho_read));
        printf("TX NCO[%d]: %.6f MHz, phase: %.3f deg\n",
               CFG_NCO_INDEX, nco_read[CFG_NCO_INDEX]/1e6, pho_read*180.0/M_PI);
    }

    /* 9) Setup TX stream */
    memset(&txs, 0, sizeof(txs));
    txs.channel  = ch;
    txs.isTx     = true;
    txs.fifoSize = CFG_FIFO_SIZE;
    txs.dataFmt  = STREAM_FMT;  /* correct for C or C++ */
    CHECK(LMS_SetupStream(dev, &txs));
    print_stream_status(&txs, "after setup");
    CHECK(LMS_StartStream(&txs));
    print_stream_status(&txs, "after start");

    /* 10) Build a DC IQ buffer (becomes a tone after NCO) */
    const int16_t I = (int16_t)(CFG_TONE_I_SCALE * 32767.0);
    const int16_t Q = (int16_t)(CFG_TONE_Q);
    const size_t  S = CFG_BUF_SAMPLES;
    int16_t* buf = (int16_t*)malloc(2*S*sizeof(int16_t));
    if (!buf) { fprintf(stderr, "malloc failed\n"); cleanup_and_exit(1); }
    for (size_t i = 0; i < S; ++i) { buf[2*i+0] = I; buf[2*i+1] = Q; }

    printf("Transmitting carrier near %.1f MHz (LO %s %.1f MHz). Ctrl+C to stop.\n",
           (lo_rd - (CFG_NCO_DOWNCONVERT ? CFG_NCO_FREQ_HZ : -CFG_NCO_FREQ_HZ))/1e6,
           CFG_NCO_DOWNCONVERT ? "-" : "+", CFG_NCO_FREQ_HZ/1e6);

    /* 11) Stream loop */
    while (1) {
        lms_stream_meta_t meta; memset(&meta, 0, sizeof(meta));
        int pushed = LMS_SendStream(&txs, buf, S, &meta, CFG_SEND_TIMEOUT_MS);
        if (pushed < 0) {
            fprintf(stderr, "LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }
        print_stream_status(&txs, "tx");
    }

    free(buf);
    cleanup_and_exit(0);
    return 0;
}
