// tx_nco_sine.c
// Generate a single RF tone using LimeSDR TX NCO.
// Key fix vs earlier: bypass TX TSP DC corrector so streamed DC is NOT cancelled.
//
// Build: gcc -O2 -Wall -Wextra -o tx_nco_sine tx_nco_sine.c -lLimeSuite -lm

#include "lime/LimeSuite.h"
#include "LMS7002M_parameters.h"   // for low-level TXTSP bypass toggles
#include <ctype.h>
#include <inttypes.h>
#include <math.h>
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
#define TONE_SCALE_DEFAULT 0.70

// clang-format off
#define CHECK(x) do { \
  int __e = (x); \
  if (__e) { fprintf(stderr,"ERROR: %s -> %s\n", #x, LMS_GetLastErrorMessage()); goto cleanup; } \
} while(0)
// clang-format on

static volatile int keep_running = 1;
static void on_sigint(int s) { (void)s; keep_running = 0; }

// ---------- helpers / prints ----------
static void print_sr(lms_device_t *dev){
    double host=0, rf=0;
    if (!LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host, &rf))
        printf("set/get: sample rate host=%.2f Msps, rf=%.2f Msps\n", host/1e6, rf/1e6);
}
static void print_gain(lms_device_t *dev){
    unsigned int g=0;
    if (!LMS_GetGaindB(dev, LMS_CH_TX, CH, &g))
        printf("set/get: TX gain = %u dB\n", g);
}
static void print_lo(lms_device_t *dev){
    double f=0;
    if (!LMS_GetLOFrequency(dev, LMS_CH_TX, CH, &f))
        printf("set/get: LO freq = %.6f MHz\n", f/1e6);
}
static void print_lpfbw(lms_device_t *dev){
    double bw=0;
    if (!LMS_GetLPFBW(dev, LMS_CH_TX, CH, &bw))
        printf("set/get: TX LPF BW = %.2f MHz\n", bw/1e6);
}
static void print_nco(lms_device_t *dev, double nco_hz, bool down){
    int idx=LMS_GetNCOIndex(dev,true,CH);
    printf("set/get: NCO idx=%d, dir=%s, set-freq=%.6f MHz (no freq readback)\n",
           idx, down?"down":"up", nco_hz/1e6);
}
static void print_txtsp_bypass(lms_device_t *dev){
    int v=0;
    if (LMS_ReadParam(dev, LMS7param(EN_TXTSP), &v)==0)
        printf("set/get: TXTSP EN=%d\n", v);
    if (LMS_ReadParam(dev, LMS7param(CMIX_BYP_TXTSP), &v)==0)
        printf("set/get: TXTSP CMIX_BYP=%d (0=enabled)\n", v);
    if (LMS_ReadParam(dev, LMS7param(DC_BYP_TXTSP), &v)==0)
        printf("set/get: TXTSP DC_BYP=%d (1=bypass)\n", v);
    if (LMS_ReadParam(dev, LMS7param(GC_BYP_TXTSP), &v)==0)
        printf("set/get: TXTSP GC_BYP=%d\n", v);
    if (LMS_ReadParam(dev, LMS7param(PH_BYP_TXTSP), &v)==0)
        printf("set/get: TXTSP PH_BYP=%d\n", v);
}
static void print_all_params(lms_device_t *dev, const char* tag, double lo_hz, double nco_hz, bool down){
    printf("---- %s ----\n", tag);
    print_sr(dev);
    print_gain(dev);
    print_lo(dev);
    print_lpfbw(dev);
    print_nco(dev, nco_hz, down);
    print_txtsp_bypass(dev);
    double host=0, rf=0; LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host, &rf);
    unsigned int g=0; LMS_GetGaindB(dev, LMS_CH_TX, CH, &g);
    const double rf_tone = down ? (lo_hz - nco_hz) : (lo_hz + nco_hz);
    printf("derived: RF tone = %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%u dB, %sconvert)\n",
           rf_tone/1e6, host/1e6, rf/1e6, g, down?"down":"up");
    printf("---------------------\n");
}

// ---------- parsing ----------
static bool parse_bool(const char* s, bool* out){
    if (!s || !out) return false;
    if (!strcasecmp(s,"1")||!strcasecmp(s,"true")||!strcasecmp(s,"yes")||!strcasecmp(s,"on")){*out=true; return true;}
    if (!strcasecmp(s,"0")||!strcasecmp(s,"false")||!strcasecmp(s,"no") ||!strcasecmp(s,"off")){*out=false;return true;}
    return false;
}
static bool parse_hz(const char* s, double* out){
    if (!s || !out) return false;
    char* end=NULL; double v=strtod(s,&end);
    if (end && *end!='\0'){
        while(*end && isspace((unsigned char)*end)) end++;
        if (*end){
            double mul=1.0; char c=(char)tolower((unsigned char)*end);
            if (c=='k') mul=1e3; else if (c=='m') mul=1e6; else if (c=='g') mul=1e9; else return false;
            end++;
            while(*end && isspace((unsigned char)*end)) end++;
            if (*end!='\0') return false;
            v*=mul;
        }
    }
    *out=v; return true;
}
static bool parse_double(const char* s, double* out){
    if (!s || !out) return false; char* e=NULL; *out=strtod(s,&e); return (e && *e=='\0');
}

// ---------- TXTSP path fix: keep DC alive ----------
static int force_tx_keep_dc(lms_device_t* dev){
    // Make sure TXTSP is enabled and complex mixer (NCO path) is active.
    int rc=0;
    rc |= LMS_WriteParam(dev, LMS7param(EN_TXTSP), 1);
    rc |= LMS_WriteParam(dev, LMS7param(CMIX_BYP_TXTSP), 0); // don't bypass CMIX
    // Bypass DC corrector so it doesn't kill our DC buffer
    rc |= LMS_WriteParam(dev, LMS7param(DC_BYP_TXTSP), 1);
    // Optional: also bypass gain/phase correctors for a raw path
    (void)LMS_WriteParam(dev, LMS7param(GC_BYP_TXTSP), 1);
    (void)LMS_WriteParam(dev, LMS7param(PH_BYP_TXTSP), 1);
    return rc;
}

int main(int argc, char** argv)
{
    // ---- defaults ----
    double HOST_SR_HZ      = 5e6;
    int    OVERSAMPLE      = 32;
    double TX_LPF_BW_HZ    = 20e6;
    double LO_HZ           = 30e6;
    double NCO_FREQ_HZ     = 1e6;
    bool   NCO_DOWNCONVERT = true;
    int    TX_GAIN_DB      = 40;
    bool   DO_CALIBRATE    = false;
    double TONE_SCALE      = TONE_SCALE_DEFAULT;
    bool   DO_RESET        = true;
    double BASEBAND_TONE_HZ= 0.0;     // 0 => DC; >0 => tiny BB tone to avoid any DC trickery

    // ---- args ----
    for (int i=1; i<argc; i++){
        const char* a = argv[i];
        #define NEEDVAL() do{ if (i+1>=argc){ fprintf(stderr,"missing value for %s\n", a); return 1; } }while(0)

        if (!strcmp(a,"--host-sr")){ NEEDVAL(); if(!parse_hz(argv[++i], &HOST_SR_HZ)) return 1; continue; }
        if (!strcmp(a,"--oversample")){ NEEDVAL(); OVERSAMPLE=(int)strtol(argv[++i],NULL,0); if(OVERSAMPLE<1) return 1; continue; }
        if (!strcmp(a,"--tx-lpf-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &TX_LPF_BW_HZ)) return 1; continue; }
        if (!strcmp(a,"--lo")){ NEEDVAL(); if(!parse_hz(argv[++i], &LO_HZ)) return 1; continue; }
        if (!strcmp(a,"--nco")){ NEEDVAL(); if(!parse_hz(argv[++i], &NCO_FREQ_HZ)) return 1; continue; }
        if (!strcmp(a,"--nco-downconvert")){ NEEDVAL(); if(!parse_bool(argv[++i], &NCO_DOWNCONVERT)) return 1; continue; }
        if (!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB=(int)strtol(argv[++i],NULL,0); continue; }
        if (!strcmp(a,"--calibrate")){ NEEDVAL(); if(!parse_bool(argv[++i], &DO_CALIBRATE)) return 1; continue; }
        if (!strcmp(a,"--tone-scale")){ NEEDVAL(); if(!parse_double(argv[++i], &TONE_SCALE) || TONE_SCALE<=0 || TONE_SCALE>1.0) return 1; continue; }
        if (!strcmp(a,"--reset")){ NEEDVAL(); if(!parse_bool(argv[++i], &DO_RESET)) return 1; continue; }
        if (!strcmp(a,"--bb-tone")){ NEEDVAL(); if(!parse_hz(argv[++i], &BASEBAND_TONE_HZ)) return 1; continue; }

        fprintf(stderr,"unknown option: %s\n", a); return 1;
    }

    // ---- setup ----
    lms_device_t *dev=NULL;
    lms_stream_t txs; memset(&txs,0,sizeof(txs));
    int16_t *buf=NULL;

    signal(SIGINT, on_sigint);

    lms_info_str_t list[8];
    int n=LMS_GetDeviceList(list);
    if (n<1){ fprintf(stderr,"no LimeSDR found\n"); return 1; }
    if (LMS_Open(&dev, list[0], NULL)){ fprintf(stderr,"LMS_Open failed: %s\n", LMS_GetLastErrorMessage()); return 1; }

    CHECK(LMS_Init(dev));
    if (DO_RESET){ CHECK(LMS_Reset(dev)); printf("device reset to defaults\n"); }

    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    printf("TX channel enabled\n");

    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));

    // Keep DC alive (bypass DC corrector, keep CMIX active)
    CHECK(force_tx_keep_dc(dev));

    // Configure NCO: magnitude via frequency table + direction via index
    {
        double freqs[16] = {0};
        freqs[NCO_INDEX] = NCO_FREQ_HZ;
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
        CHECK(LMS_SetNCOIndex(dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
    }

    // Always print pre-calibration snapshot
    print_all_params(dev, "pre-calibration state (no changes yet)", LO_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);

    // Optional calibration (off by default)
    if (DO_CALIBRATE){
        CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ, 0));
        printf("TX calibrated (bw=%.2f MHz)\n", TX_LPF_BW_HZ/1e6);
        // Keep DC bypass ON after calibration (it may be re-enabled by cal)
        (void)force_tx_keep_dc(dev);
    } else {
        printf("TX calibration skipped (use --calibrate on to enable)\n");
    }

    // Always print post-calibration snapshot
    print_all_params(dev, "post-calibration state (current settings)", LO_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);

    // ---- stream ----
    txs.channel = CH;
    txs.isTx = true;
    txs.fifoSize = FIFO_SIZE_SAMPLES;
    txs.dataFmt = LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev, &txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16)\n", FIFO_SIZE_SAMPLES);

    // Prepare baseband buffer:
    //  - if BASEBAND_TONE_HZ==0 -> constant DC (I=const, Q=0) -> NCO makes RF sine at LO±NCO
    //  - else tiny baseband tone at fb -> appears at LO±NCO±fb (handy if you ever want to avoid DC fully)
    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if (!buf){ fprintf(stderr,"malloc failed\n"); goto cleanup; }

    double host_sr=0, rf_sr=0; LMS_GetSampleRate(dev, LMS_CH_TX, CH, &host_sr, &rf_sr);
    const int16_t AMPL = (int16_t)(TONE_SCALE_DEFAULT * 32767.0);

    if (BASEBAND_TONE_HZ <= 0.0){
        for (int i=0;i<BUF_SAMPLES;i++){ buf[2*i+0]=AMPL; buf[2*i+1]=0; }
    } else {
        // tiny baseband tone (complex)
        for (int i=0;i<BUF_SAMPLES;i++){
            double t = (double)i / host_sr;           // host-side sample rate
            double ph = 2.0*M_PI*BASEBAND_TONE_HZ*t;
            buf[2*i+0] = (int16_t)(AMPL * cos(ph));
            buf[2*i+1] = (int16_t)(AMPL * sin(ph));
        }
    }

    const double rf_hz = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
    printf("TX tone target %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%d dB, %sconvert)\n",
           rf_hz/1e6, host_sr/1e6, rf_sr/1e6, TX_GAIN_DB, NCO_DOWNCONVERT?"down":"up");
    printf("Ctrl+C to stop\n");

    while (keep_running){
        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if (LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0){
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }
    }
    printf("\nSIGINT detected\n");

cleanup:
    if (txs.handle){
        int16_t* z = (int16_t*)calloc(2*BUF_SAMPLES, sizeof(int16_t));
        if (z){
            lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
            (void)LMS_SendStream(&txs, z, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS);
            free(z);
        }
        LMS_StopStream(&txs);
        LMS_DestroyStream(dev, &txs);
        printf("TX stream stopped\n");
    }
    if (dev){
        LMS_EnableChannel(dev, LMS_CH_TX, CH, false);
        printf("TX channel disabled\n");
        LMS_Close(dev);
    }
    if (buf) free(buf);
    return 0;
}
