// tx_realtime_from_python.c
// Realtime version of: x = sin(Ω t) + j*cos(Ω t)  (Ω in rad/s)
// Streams a zero-mean complex tone -> works with default TXTSP DC removal (no bypass).
//
// Resulting RF tone (single sideband) appears at:  RF = LO ± (NCO ∓ f_bb)
// The "∓" is because sin+ j*cos is a NEGATIVE-rotating complex tone.
// Build: gcc -O2 -Wall -Wextra -o tx_realtime_from_python tx_realtime_from_python.c -lLimeSuite -lm

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
#define NCO_INDEX 0
#define FIFO_SIZE_SAMPLES (1<<17)
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
static void on_sigint(int s){ (void)s; keep_running = 0; }

// ---------- prints ----------
static void print_sr(lms_device_t *dev){
    double host=0, rf=0; if(!LMS_GetSampleRate(dev,LMS_CH_TX,CH,&host,&rf))
        printf("set/get: sample rate host=%.2f Msps, rf=%.2f Msps\n", host/1e6, rf/1e6);
}
static void print_gain(lms_device_t *dev){
    unsigned g=0; if(!LMS_GetGaindB(dev,LMS_CH_TX,CH,&g))
        printf("set/get: TX gain = %u dB\n", g);
}
static void print_lo(lms_device_t *dev){
    double f=0; if(!LMS_GetLOFrequency(dev,LMS_CH_TX,CH,&f))
        printf("set/get: LO freq = %.6f MHz\n", f/1e6);
}
static void print_lpfbw(lms_device_t *dev){
    double bw=0; if(!LMS_GetLPFBW(dev,LMS_CH_TX,CH,&bw))
        printf("set/get: TX LPF BW = %.2f MHz\n", bw/1e6);
}
static void print_nco(lms_device_t *dev, double nco_hz, bool down){
    int idx=LMS_GetNCOIndex(dev,true,CH);
    printf("set/get: NCO idx=%d, dir=%s, set-freq=%.6f MHz (no freq readback)\n",
           idx, down?"down":"up", nco_hz/1e6);
}
static void print_all(lms_device_t *dev, const char* tag, double lo_hz, double nco_hz, bool down){
    printf("---- %s ----\n", tag);
    print_sr(dev); print_gain(dev); print_lo(dev); print_lpfbw(dev); print_nco(dev,nco_hz,down);
    double host=0, rf=0; LMS_GetSampleRate(dev,LMS_CH_TX,CH,&host,&rf);
    unsigned g=0; LMS_GetGaindB(dev,LMS_CH_TX,CH,&g);
    printf("derived: LO=%.6f MHz, NCO=%.6f MHz (%sconvert), host=%.2f Msps, rf=%.2f Msps, gain=%u dB\n",
           lo_hz/1e6, nco_hz/1e6, down?"down":"up", host/1e6, rf/1e6, g);
    printf("---------------------\n");
}

// ---------- parse ----------
static bool parse_bool(const char* s, bool* out){
    if(!s||!out) return false;
    if (!strcasecmp(s,"1")||!strcasecmp(s,"true")||!strcasecmp(s,"yes")||!strcasecmp(s,"on"))  { *out=true;  return true; }
    if (!strcasecmp(s,"0")||!strcasecmp(s,"false")||!strcasecmp(s,"no") ||!strcasecmp(s,"off")){ *out=false; return true; }
    return false;
}
static bool parse_hz(const char* s, double* out){
    if (!s||!out) return false;
    char* end=NULL; double v=strtod(s,&end);
    if (end && *end!='\0'){
        while(*end && isspace((unsigned char)*end)) end++;
        if (*end){
            double m=1.0; char c=(char)tolower((unsigned char)*end);
            if(c=='k') m=1e3; else if(c=='m') m=1e6; else if(c=='g') m=1e9; else return false;
            end++; while(*end && isspace((unsigned char)*end)) end++;
            if(*end!='\0') return false; v*=m;
        }
    }
    *out=v; return true;
}
static bool parse_double(const char* s, double* out){ if(!s||!out) return false; char* e=NULL; *out=strtod(s,&e); return (e && *e=='\0'); }

// ---------- main ----------
int main(int argc, char** argv)
{
    // Defaults (match your Python)
    double HOST_SR_HZ      = 5e6;    // WAV_SMP
    int    OVERSAMPLE      = 32;
    double TX_LPF_BW_HZ    = 50e6;
    double LO_HZ           = 50e6;
    double NCO_FREQ_HZ     = 38.5e6;
    bool   NCO_DOWNCONVERT = true;
    int    TX_GAIN_DB      = 73;
    bool   DO_CALIBRATE    = false;

    // Python tone parameters
    double BB_ANG_RAD_S    = 2048.0; // Ω
    double BB_FREQ_HZ      = -1.0;   // optional direct Hz override (wins if >0)
    double TONE_SCALE      = TONE_SCALE_DEFAULT;

    for (int i=1;i<argc;i++){
        const char* a=argv[i];
        #define NEEDVAL() do{ if(i+1>=argc){ fprintf(stderr,"missing value for %s\n", a); return 1; } }while(0)
        if(!strcmp(a,"--host-sr")){ NEEDVAL(); if(!parse_hz(argv[++i],&HOST_SR_HZ)) return 1; continue; }
        if(!strcmp(a,"--oversample")){ NEEDVAL(); OVERSAMPLE=(int)strtol(argv[++i],NULL,0); continue; }
        if(!strcmp(a,"--tx-lpf-bw")){ NEEDVAL(); if(!parse_hz(argv[++i],&TX_LPF_BW_HZ)) return 1; continue; }
        if(!strcmp(a,"--lo")){ NEEDVAL(); if(!parse_hz(argv[++i],&LO_HZ)) return 1; continue; }
        if(!strcmp(a,"--nco")){ NEEDVAL(); if(!parse_hz(argv[++i],&NCO_FREQ_HZ)) return 1; continue; }
        if(!strcmp(a,"--nco-downconvert")){ NEEDVAL(); if(!parse_bool(argv[++i],&NCO_DOWNCONVERT)) return 1; continue; }
        if(!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB=(int)strtol(argv[++i],NULL,0); continue; }
        if(!strcmp(a,"--calibrate")){ NEEDVAL(); if(!parse_bool(argv[++i],&DO_CALIBRATE)) return 1; continue; }
        if(!strcmp(a,"--bb-ang")){ NEEDVAL(); if(!parse_double(argv[++i],&BB_ANG_RAD_S)) return 1; continue; }
        if(!strcmp(a,"--bb-hz")){ NEEDVAL(); if(!parse_hz(argv[++i],&BB_FREQ_HZ)) return 1; continue; }
        if(!strcmp(a,"--tone-scale")){ NEEDVAL(); if(!parse_double(argv[++i],&TONE_SCALE) || TONE_SCALE<=0 || TONE_SCALE>1.0) return 1; continue; }
        fprintf(stderr,"unknown option: %s\n", a); return 1;
    }

    // Compute baseband tone frequency (Hz)
    const double f_bb = (BB_FREQ_HZ>0)? BB_FREQ_HZ : (BB_ANG_RAD_S/(2.0*M_PI));
    if (f_bb <= 0.0){
        fprintf(stderr,"WARNING: baseband frequency is %.3f Hz; DC or negative not intended. Use --bb-hz > 0.\n", f_bb);
    }

    // Open Lime
    lms_device_t *dev=NULL;
    lms_stream_t txs; memset(&txs,0,sizeof(txs));
    int16_t* buf=NULL;

    signal(SIGINT, on_sigint);

    lms_info_str_t list[8];
    int n=LMS_GetDeviceList(list);
    if(n<1){ fprintf(stderr,"no LimeSDR found\n"); return 1; }
    if(LMS_Open(&dev, list[0], NULL)){ fprintf(stderr,"LMS_Open failed: %s\n", LMS_GetLastErrorMessage()); return 1; }

    CHECK(LMS_Init(dev));
    CHECK(LMS_EnableChannel(dev, LMS_CH_TX, CH, true));
    CHECK(LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE));
    CHECK(LMS_SetLPFBW(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ));
    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB));
    CHECK(LMS_SetLOFrequency(dev, LMS_CH_TX, CH, LO_HZ));

    // NCO
    {
        double freqs[16]={0}; freqs[NCO_INDEX]=NCO_FREQ_HZ;
        CHECK(LMS_SetNCOFrequency(dev, true, CH, freqs, 0.0));
        CHECK(LMS_SetNCOIndex(dev, true, CH, NCO_INDEX, NCO_DOWNCONVERT));
    }

    // Always print pre-/post- snapshots (calibration off by default)
    print_all(dev, "pre-calibration state (no changes yet)", LO_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);
    if(DO_CALIBRATE){
        CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, TX_LPF_BW_HZ, 0));
        printf("TX calibrated (bw=%.2f MHz)\n", TX_LPF_BW_HZ/1e6);
    } else {
        printf("TX calibration skipped (use --calibrate on to enable)\n");
    }
    print_all(dev, "post-calibration state (current settings)", LO_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT);

    // Stream setup
    txs.channel=CH; txs.isTx=true; txs.fifoSize=FIFO_SIZE_SAMPLES; txs.dataFmt=LMS_FMT_I16;
    CHECK(LMS_SetupStream(dev,&txs));
    CHECK(LMS_StartStream(&txs));
    printf("TX stream started (fifo=%d samples, fmt=I16)\n", FIFO_SIZE_SAMPLES);

    // -------- generate: x = sin(Ω t) + j*cos(Ω t) --------
    // Make a DDS/rotator at +f_bb using complex unit u=cos+ j sin.
    // Then map to Python's layout: I = sin = Im(u), Q = cos = Re(u).
    buf = (int16_t*)malloc(2*BUF_SAMPLES*sizeof(int16_t));
    if(!buf){ fprintf(stderr,"malloc failed\n"); goto cleanup; }

    const double step = 2.0*M_PI * f_bb / HOST_SR_HZ;  // radians per sample
    const double wr = cos(step), wi = sin(step);       // rotator coefficient
    double ur = 1.0, ui = 0.0;                         // u = e^{jθ}
    const int16_t AMP = (int16_t)(TONE_SCALE_DEFAULT * 32767.0);

    double host=0, rf=0; LMS_GetSampleRate(dev,LMS_CH_TX,CH,&host,&rf);
    // Because I=sin, Q=cos -> NEGATIVE rotating baseband tone.
    // Final RF tone location:
    const double rf_center = NCO_DOWNCONVERT ? (LO_HZ - NCO_FREQ_HZ) : (LO_HZ + NCO_FREQ_HZ);
    const double rf_tone   = rf_center - f_bb; // negative rotation -> subtract f_bb
    printf("SIGNAL: python-style complex tone (I=sin, Q=cos)\n");
    printf("       f_bb = %.6f Hz (Ω=%.3f rad/s), host-sr = %.3f Msps, amp = %.2f FS\n",
           f_bb, (BB_FREQ_HZ>0? 2*M_PI*BB_FREQ_HZ: BB_ANG_RAD_S), host/1e6, TONE_SCALE);
    printf("       mixing: %sconvert via NCO=%.6f MHz\n",
           NCO_DOWNCONVERT?"down":"up", NCO_FREQ_HZ/1e6);
    printf("       RF result: LO±NCO∓f_bb -> %.6f MHz (center %.6f MHz, minus f_bb)\n",
           rf_tone/1e6, rf_center/1e6);
    printf("Ctrl+C to stop\n");

    // Main loop
    while(keep_running){
        // fill buffer
        for(int i=0;i<BUF_SAMPLES;i++){
            // python mapping: I=sin(θ)=Im(u), Q=cos(θ)=Re(u)
            buf[2*i+0] = (int16_t)lrint(AMP * ui); // I
            buf[2*i+1] = (int16_t)lrint(AMP * ur); // Q

            // rotate u <- u * w
            const double nr = ur*wr - ui*wi;
            const double ni = ur*wi + ui*wr;
            ur = nr; ui = ni;

            // occasional renormalize to limit drift
            if((i & 1023) == 1023){
                const double r2 = ur*ur + ui*ui;
                const double k  = 1.5 - 0.5*r2; // one-step NR approx to 1/sqrt(r2)
                ur *= k; ui *= k;
            }
        }

        lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
        if(LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS) < 0){
            fprintf(stderr,"LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
            break;
        }
    }

    printf("\nSIGINT detected\n");

cleanup:
    if(txs.handle){
        int16_t* z=(int16_t*)calloc(2*BUF_SAMPLES,sizeof(int16_t));
        if(z){
            lms_stream_meta_t meta; memset(&meta,0,sizeof(meta));
            (void)LMS_SendStream(&txs, z, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS);
            free(z);
        }
        LMS_StopStream(&txs);
        LMS_DestroyStream(dev,&txs);
        printf("TX stream stopped\n");
    }
    if(dev){ LMS_EnableChannel(dev,LMS_CH_TX,CH,false); LMS_Close(dev); printf("TX channel disabled\n"); }
    if(buf) free(buf);
    return 0;
}
