// lime_test.c — LimeSDR TX test patterns for tinySA measurement (pure C)
//
// Build: gcc -O2 -Wall -Wextra -o lime_test lime_test.c -lLimeSuite -lm

#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <inttypes.h>
#include <stdbool.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- Defaults ----------
#define CH                0              // TX channel A
#define HOST_SR_HZ        5000000        // 5 Msps USB
#define OVERSAMPLE        8              // RF SR ≈ 40 Msps (NCO range up to ~20 MHz)
#define TX_LPF_BW_HZ      50000000       // 50 MHz LPF
#define LO_HZ             30000000       // 30 MHz LO (PLL-friendly)
#define TX_GAIN_DB        40
#define FIFO_SIZE_SAMPLES (1<<17)
#define BUF_SAMPLES       8192
#define SEND_TIMEOUT_MS   1000

// ---------- Helpers ----------
#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)

static volatile int g_run = 1;
static void on_sigint(int s){ (void)s; g_run = 0; }

typedef enum { MODE_TONE, MODE_TWOTONE, MODE_SWEEP, MODE_NOISE } tx_mode_t;

typedef struct {
    tx_mode_t mode;
    double rf_target_hz;       // for tone/twotone/noise
    double amp_fs;             // 0..1 total headroom
    unsigned gain_db;
    // two-tone
    double tone_delta_hz;
    // sweep
    double sweep_start_hz, sweep_stop_hz, sweep_step_hz;
    int dwell_ms;
    // noise
    double noise_bw_hz;        // (PRBS-based; not strictly enforced)
} cfg_t;

static void usage(const char* exe){
    fprintf(stderr,
    "Usage: %s --mode {tone|twotone|sweep|noise} [options]\n"
    "Common:\n"
    "  --gain <dB>           (default %u)\n"
    "  --amp <0..1>          (default 0.70)\n"
    "Tone/Two-tone/Noise:\n"
    "  --rf <Hz>             target RF (e.g. 15e6)\n"
    "Two-tone:\n"
    "  --tone-delta <Hz>     spacing from center (default 50e3)\n"
    "Sweep:\n"
    "  --sweep-start <Hz> --sweep-stop <Hz> --sweep-step <Hz>\n"
    "  --dwell-ms <ms>       per-step dwell (default 30)\n"
    "Noise:\n"
    "  --noise-bw <Hz>       approximate occupied BW (default 1e6)\n",
    exe, TX_GAIN_DB);
}

static bool parse_args(int argc, char** argv, cfg_t* c){
    // defaults
    memset(c,0,sizeof(*c));
    c->mode = MODE_TONE;
    c->rf_target_hz = 15e6;
    c->amp_fs = 0.70;
    c->gain_db = TX_GAIN_DB;
    c->tone_delta_hz = 50e3;
    c->sweep_start_hz = 10e6; c->sweep_stop_hz = 30e6; c->sweep_step_hz = 100e3; c->dwell_ms = 30;
    c->noise_bw_hz = 1e6;

    for (int i=1;i<argc;i++){
        if (!strcmp(argv[i],"--mode") && i+1<argc){
            i++;
            if (!strcmp(argv[i],"tone")) c->mode=MODE_TONE;
            else if (!strcmp(argv[i],"twotone")) c->mode=MODE_TWOTONE;
            else if (!strcmp(argv[i],"sweep")) c->mode=MODE_SWEEP;
            else if (!strcmp(argv[i],"noise")) c->mode=MODE_NOISE;
            else { usage(argv[0]); return false; }
        } else if (!strcmp(argv[i],"--rf") && i+1<argc){
            c->rf_target_hz = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--gain") && i+1<argc){
            c->gain_db = (unsigned)atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--amp") && i+1<argc){
            c->amp_fs = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--tone-delta") && i+1<argc){
            c->tone_delta_hz = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--sweep-start") && i+1<argc){
            c->sweep_start_hz = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--sweep-stop") && i+1<argc){
            c->sweep_stop_hz = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--sweep-step") && i+1<argc){
            c->sweep_step_hz = atof(argv[++i]);
        } else if (!strcmp(argv[i],"--dwell-ms") && i+1<argc){
            c->dwell_ms = atoi(argv[++i]);
        } else if (!strcmp(argv[i],"--noise-bw") && i+1<argc){
            c->noise_bw_hz = atof(argv[++i]);
        } else {
            usage(argv[0]); return false;
        }
    }
    return true;
}

// ----- Print-back helpers -----
static void print_sr(lms_device_t* dev){
    double host=0, rf=0;
    if (!LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host, &rf))
        printf("Set/Get: SampleRate host=%.3f Msps, rf=%.3f Msps\n", host/1e6, rf/1e6);
}
static void print_gain(lms_device_t* dev){
    unsigned int g = 0;
    if (!LMS_GetGaindB(dev, LMS_CH_TX, CH, &g))
        printf("Set/Get: TX Gain = %u dB\n", g);
}
static void print_lpf(lms_device_t* dev){
    float_type bw = 0;
    if (!LMS_GetLPFBW(dev, LMS_CH_TX, CH, &bw))
        printf("Set/Get: TX LPF BW = %.2f MHz\n", bw/1e6);
}
static void print_lo(lms_device_t* dev){
    double lof = 0;
    if (!LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &lof))
        printf("Set/Get: LO = %.6f MHz\n", lof/1e6);
}
static void print_nco(lms_device_t* dev){
    int idx = LMS_GetNCOIndex(dev, true, CH);
    if (idx >= 0){
        double freqs[16]={0}; double ph=0.0;
        if (!LMS_GetNCOFrequency(dev, true, CH, freqs, &ph))
            printf("Set/Get: NCO idx=%d, f=%.6f MHz, mode=%s\n", idx, freqs[idx]/1e6, "downconv");
    }
}

// ----- Generators -----
static void fill_two_tone_iq(int16_t* i16, size_t n, double amp, double w1, double w2){
    double ph1=0.0, ph2=0.0;
    double a = amp * 0.5; // split amplitude
    for (size_t k=0;k<n;k++){
        float s = (float)(a*sin(ph1) + a*sin(ph2));
        i16[2*k+0] = (int16_t)lrintf(s * 32767.0f);
        i16[2*k+1] = 0;
        ph1 += w1; if (ph1>2*M_PI) ph1-=2*M_PI;
        ph2 += w2; if (ph2>2*M_PI) ph2-=2*M_PI;
    }
}
static uint32_t xorshift32(uint32_t x){ x ^= x<<13; x ^= x>>17; x ^= x<<5; return x; }
static void fill_noise_iq(int16_t* i16, size_t n, double amp){
    static uint32_t s1=0x12345678, s2=0x87654321;
    for (size_t k=0;k<n;k++){
        s1 = xorshift32(s1); s2 = xorshift32(s2);
        float fi = ((int32_t)(s1 & 0xFFFF) - 32768) / 32768.0f;
        float fq = ((int32_t)(s2 & 0xFFFF) - 32768) / 32768.0f;
        i16[2*k+0] = (int16_t)lrintf((float)amp * fi * 32767.0f);
        i16[2*k+1] = (int16_t)lrintf((float)amp * fq * 32767.0f);
    }
}

// Sleep helper
static void msleep(int ms){
    struct timespec ts; ts.tv_sec = ms/1000; ts.tv_nsec = (ms%1000)*1000000L; nanosleep(&ts, NULL);
}

// Program NCO to get desired RF (downconvert mode around fixed LO)
static int set_rf(lms_device_t* dev, double rf_sr, double rf_hz){
    double nco = LO_HZ - rf_hz; // want RF = LO - NCO
    double nco_max = rf_sr/2.0 - 1.0; // guard
    if (fabs(nco) > nco_max){
        fprintf(stderr,"RF %.3f MHz out of NCO range (|NCO| <= %.3f MHz)\n", rf_hz/1e6, nco_max/1e6);
        return -1;
    }
    double freqs[16] = {0};
    freqs[0] = fabs(nco);
    if (LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0)) return -1;
    // downconvert flag = true when NCO subtracts from LO
    bool downconvert = (nco >= 0);
    if (LMS_SetNCOIndex(dev, true, CH, 0, downconvert)) return -1;
    print_nco(dev);
    return 0;
}

int main(int argc, char** argv){
    cfg_t C;
    if (!parse_args(argc, argv, &C)) return 1;

    signal(SIGINT, on_sigint);

    lms_device_t* dev = NULL;
    lms_stream_t  txs; memset(&txs,0,sizeof(txs));
    int16_t*      buf = NULL;

    // ----- Open device -----
    lms_info_str_t list[8];
    int ndev = LMS_GetDeviceList(list);
    if (ndev < 1) { fprintf(stderr,"No LimeSDR found\n"); return 1; }
    if (LMS_Open(&dev, list[0], NULL)) { fprintf(stderr,"LMS_Open failed: %s\n", LMS_GetLastErrorMessage()); return 1; }

    // ----- Configure -----
    CHECK(LMS_Init(dev));

    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    printf("TX channel enabled.\n");

    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    print_sr(dev);
    double host_sr=0, rf_sr=0; LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);

    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));
    print_lpf(dev);

    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, C.gain_db));
    print_gain(dev);

    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));
    print_lo(dev);

    if (C.mode != MODE_SWEEP){
        if (set_rf(dev, rf_sr, C.rf_target_hz) != 0) goto cleanup;
    }

    // ----- Stream -----
    txs.channel  = CH;
    txs.isTx     = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt  = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d, fmt=I16)\n", FIFO_SIZE_SAMPLES);

    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf){ fprintf(stderr,"malloc failed\n"); goto cleanup; }

    printf("Mode: %s\n",
        C.mode==MODE_TONE?"tone":C.mode==MODE_TWOTONE?"twotone":C.mode==MODE_SWEEP?"sweep":"noise");

    if (C.mode == MODE_TONE){
        // Constant DC IQ -> CW at RF after NCO
        const int16_t I = (int16_t)lrint(C.amp_fs * 32767.0);
        const int16_t Q = 0;
        for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=I; buf[2*i+1]=Q; }
        printf("TX: CW @ %.6f MHz, gain=%u dB, amp=%.2f FS. Ctrl+C to stop.\n",
               C.rf_target_hz/1e6, C.gain_db, C.amp_fs);
        while (g_run){
            lms_stream_meta_t m; memset(&m,0,sizeof(m));
            if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &m, SEND_TIMEOUT_MS) < 0){
                fprintf(stderr,"LMS_SendStream: %s\n", LMS_GetLastErrorMessage()); break;
            }
        }
    }
    else if (C.mode == MODE_TWOTONE){
        double w1 = 2.0*M_PI*(+C.tone_delta_hz)/host_sr;
        double w2 = 2.0*M_PI*(-C.tone_delta_hz)/host_sr;
        printf("TX: Two-tone @ RF=%.6f MHz (±%.0f Hz), gain=%u dB, total amp=%.2f FS. Ctrl+C to stop.\n",
               C.rf_target_hz/1e6, C.tone_delta_hz, C.gain_db, C.amp_fs);
        while (g_run){
            fill_two_tone_iq(buf, BUF_SAMPLES, C.amp_fs, w1, w2);
            lms_stream_meta_t m; memset(&m,0,sizeof(m));
            if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &m, SEND_TIMEOUT_MS) < 0){
                fprintf(stderr,"LMS_SendStream: %s\n", LMS_GetLastErrorMessage()); break;
            }
        }
    }
    else if (C.mode == MODE_SWEEP){
        const int16_t I = (int16_t)lrint(0.70 * 32767.0);
        const int16_t Q = 0;
        for (size_t i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=I; buf[2*i+1]=Q; }
        printf("TX: Sweep %.3f → %.3f MHz, step %.3f kHz, dwell %d ms. Ctrl+C to stop.\n",
               C.sweep_start_hz/1e6, C.sweep_stop_hz/1e6, C.sweep_step_hz/1e3, C.dwell_ms);

        for (double rf=C.sweep_start_hz; g_run; rf += C.sweep_step_hz){
            if (rf > C.sweep_stop_hz) rf = C.sweep_start_hz;
            if (set_rf(dev, rf_sr, rf) != 0) break;

            // Dwell: push a few buffers
            int t = 0;
            while (g_run && t < C.dwell_ms){
                lms_stream_meta_t m; memset(&m,0,sizeof(m));
                if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &m, SEND_TIMEOUT_MS) < 0){
                    fprintf(stderr,"LMS_SendStream: %s\n", LMS_GetLastErrorMessage()); g_run=0; break;
                }
                double ms = (1000.0 * BUF_SAMPLES) / host_sr;
                t += (int)ms;
                if (t < C.dwell_ms) msleep(1);
            }
        }
    }
    else { // MODE_NOISE
        printf("TX: Noise centered at %.6f MHz, ~BW=%.0f kHz, gain=%u dB, amp=%.2f FS. Ctrl+C to stop.\n",
               C.rf_target_hz/1e6, C.noise_bw_hz/1e3, C.gain_db, C.amp_fs);
        while (g_run){
            fill_noise_iq(buf, BUF_SAMPLES, C.amp_fs);
            lms_stream_meta_t m; memset(&m,0,sizeof(m));
            if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &m, SEND_TIMEOUT_MS) < 0){
                fprintf(stderr,"LMS_SendStream: %s\n", LMS_GetLastErrorMessage()); break;
            }
        }
    }

    printf("\nStopping…\n");

cleanup:
    // Mute: send a zero buffer once, then stop/destroy, disable channel
    if (txs.handle){
        int16_t* z = (int16_t*)calloc(2*BUF_SAMPLES, sizeof(int16_t));
        if (z){
            lms_stream_meta_t m; memset(&m,0,sizeof(m));
            (void)LMS_SendStream(&txs, z, BUF_SAMPLES, &m, SEND_TIMEOUT_MS);
            free(z);
        }
        LMS_StopStream(&txs);
        LMS_DestroyStream(dev, &txs);
        printf("TX stream stopped.\n");
    }
    if (dev){
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled.\n");
        LMS_Close(dev);
    }
    if (buf) free(buf);
    return 0;
}
