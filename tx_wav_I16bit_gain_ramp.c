#include "lime/LimeSuite.h"
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#define CH 0
#define NCO_INDEX 0
#define FIFO_SIZE_SAMPLES (1 << 17)
#define BUF_SAMPLES 8192
#define SEND_TIMEOUT_MS 1000

#define TX_GAIN_MIN_DB 0
#define TX_GAIN_MAX_DB 73

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

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000ull);
}

#pragma pack(push, 1)
typedef struct {
    char id[4];
    uint32_t size;
} chunk_hdr_t;
#pragma pack(pop)

static bool read_exact(FILE *f, void *p, size_t n) { return fread(p, 1, n, f) == n; }
static bool read_chunk_hdr(FILE *f, chunk_hdr_t *h) { return read_exact(f, h, sizeof(*h)); }
static int str4eq(const char id[4], const char *s) {
    return id[0] == s[0] && id[1] == s[1] && id[2] == s[2] && id[3] == s[3];
}

typedef struct {
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t channels;
    uint64_t data_offset;
    uint64_t data_bytes;
} wav_info_t;

static bool parse_wav(const char *path, wav_info_t *wi, FILE **outf) {
    memset(wi, 0, sizeof(*wi));
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "Failed to open WAV: %s\n", strerror(errno));
        return false;
    }

    chunk_hdr_t h;
    if (!read_chunk_hdr(f, &h) || !str4eq(h.id, "RIFF")) {
        fprintf(stderr, "Not a RIFF file\n");
        fclose(f);
        return false;
    }

    char wave[4] = {0};
    if (!read_exact(f, wave, 4) || !str4eq(wave, "WAVE")) {
        fprintf(stderr, "Not a WAVE file\n");
        fclose(f);
        return false;
    }

    bool got_fmt = false, got_data = false;
    uint16_t audio_format = 0;
    long data_pos = 0;
    uint32_t data_size = 0;

    while (read_chunk_hdr(f, &h)) {
        if (str4eq(h.id, "fmt ")) {
            uint8_t fmtbuf[64] = {0};
            size_t toread = h.size < sizeof(fmtbuf) ? h.size : sizeof(fmtbuf);
            if (!read_exact(f, fmtbuf, toread)) {
                fprintf(stderr, "WAV: short read in fmt\n");
                fclose(f);
                return false;
            }
            if (h.size > toread)
                fseek(f, (long)(h.size - toread), SEEK_CUR);

            audio_format = (uint16_t)(fmtbuf[0] | (fmtbuf[1] << 8));
            wi->channels = (uint16_t)(fmtbuf[2] | (fmtbuf[3] << 8));
            wi->sample_rate = (uint32_t)(fmtbuf[4] | (fmtbuf[5] << 8) | (fmtbuf[6] << 16) | (fmtbuf[7] << 24));
            wi->bits_per_sample = (uint16_t)(fmtbuf[14] | (fmtbuf[15] << 8));
            got_fmt = true;
        } else if (str4eq(h.id, "data")) {
            data_pos = ftell(f);
            data_size = h.size;
            fseek(f, (long)h.size, SEEK_CUR);
            got_data = true;
        } else {
            fseek(f, (long)h.size, SEEK_CUR);
        }
        if (got_fmt && got_data)
            break;
    }

    if (!got_fmt || !got_data) {
        fprintf(stderr, "WAV: missing %s chunk\n", got_fmt ? "data" : "fmt ");
        fclose(f);
        return false;
    }
    if (!(audio_format == 1 || audio_format == 0xFFFE)) { // PCM or extensible PCM
        fprintf(stderr, "WAV: unsupported AudioFormat=0x%04x (need PCM=1)\n", audio_format);
        fclose(f);
        return false;
    }
    if (wi->channels != 2 || wi->bits_per_sample != 16) {
        fprintf(stderr, "WAV: need stereo(2ch) 16-bit; got %u ch, %u bits\n", wi->channels, wi->bits_per_sample);
        fclose(f);
        return false;
    }

    wi->data_offset = (uint64_t)data_pos;
    wi->data_bytes = (uint64_t)data_size;
    fseek(f, (long)wi->data_offset, SEEK_SET);
    *outf = f;
    return true;
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
    double TX_LPF_BW_HZ = 20e6;
    double LO_HZ = 30e6;
    double NCO_FREQ_HZ = 15e6;
    bool NCO_DOWNCONVERT = true;
    int TX_GAIN_DB = 40;
    double CAL_BW_HZ = -1;

    int TX_GAIN_START = 40;
    int RAMP_MS = 0;              // 0 = ramp disabled (old behavior)
    int RAMP_INTERVAL_MS = 20;    // step interval when ramp enabled

    const char *WAV_PATH = NULL;
    bool LOOP = false;
    double SCALE = 1.0;

    bool DO_RESET = false;
    bool DO_CALIBRATE = false;

    bool PRINT_CORRECTORS = false;
    bool SET_GI = false, SET_GQ = false, SET_PHASE = false, SET_DCI = false, SET_DCQ = false;
    int MAN_GI = 0, MAN_GQ = 0, MAN_PHASE = 0, MAN_DCI = 0, MAN_DCQ = 0;
    bool HAVE_TX_GAIN_START = false;

    // clang-format off
    for (int i=1; i<argc; i++){
        const char* a = argv[i];
        #define NEEDVAL() do{ if (i+1>=argc){ fprintf(stderr,"missing value for %s\n", a); return 1; } }while(0)

        if (!strcmp(a,"--file")){ NEEDVAL(); WAV_PATH = argv[++i]; continue; }
        if (!strcmp(a,"--oversample")){ NEEDVAL(); OVERSAMPLE = (int)strtol(argv[++i], NULL, 0); if (OVERSAMPLE<1){ fprintf(stderr,"bad --oversample\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-lpf-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &TX_LPF_BW_HZ)) { fprintf(stderr,"bad --tx-lpf-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--lo")){ NEEDVAL(); if(!parse_hz(argv[++i], &LO_HZ)) { fprintf(stderr,"bad --lo\n"); return 1; } continue; }
        if (!strcmp(a,"--nco")){ NEEDVAL(); if(!parse_hz(argv[++i], &NCO_FREQ_HZ)) { fprintf(stderr,"bad --nco\n"); return 1; } continue; }
        if (!strcmp(a,"--nco-downconvert")){ NEEDVAL(); if(!parse_bool(argv[++i], &NCO_DOWNCONVERT)) { fprintf(stderr,"bad --nco-downconvert\n"); return 1; } continue; }
        if (!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB = (int)strtol(argv[++i], NULL, 0); continue; }
        if (!strcmp(a,"--tx-gain-start")){ NEEDVAL(); TX_GAIN_START = (int)strtol(argv[++i], NULL, 0); HAVE_TX_GAIN_START = true; continue; }
        if (!strcmp(a,"--gain-ramp-ms")){ NEEDVAL(); RAMP_MS = (int)strtol(argv[++i], NULL, 0); continue; }
        if (!strcmp(a,"--gain-ramp-interval-ms")){ NEEDVAL(); RAMP_INTERVAL_MS = (int)strtol(argv[++i], NULL, 0); continue; }
        if (!strcmp(a,"--cal-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &CAL_BW_HZ)) { fprintf(stderr,"bad --cal-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--loop")){ LOOP = true; continue; }
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
    if (!WAV_PATH){ fprintf(stderr,"missing --file <path.wav>\n"); return 1; }
    if (CAL_BW_HZ <= 0) CAL_BW_HZ = TX_LPF_BW_HZ;

    TX_GAIN_DB    = clampi(TX_GAIN_DB,    TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);
    if (!HAVE_TX_GAIN_START)
        TX_GAIN_START = TX_GAIN_DB;
    TX_GAIN_START = clampi(TX_GAIN_START, TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);
    if (RAMP_INTERVAL_MS < 1) RAMP_INTERVAL_MS = 1;
    if (RAMP_MS < 0) RAMP_MS = 0;
    // clang-format on

    signal(SIGINT, on_sigint);

    wav_info_t wi;
    FILE *wf = NULL;
    if (!parse_wav(WAV_PATH, &wi, &wf))
        return 1;
    const double HOST_SR_HZ = (double)wi.sample_rate;

    printf("WAV: %u Hz, %u-bit, %u ch, data=%" PRIu64 " bytes @ 0x%08" PRIx64 "\n", wi.sample_rate, wi.bits_per_sample,
           wi.channels, wi.data_bytes, wi.data_offset);

    lms_device_t *dev = NULL;
    lms_stream_t txs;
    int16_t *buf = NULL;
    memset(&txs, 0, sizeof(txs));

    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) {
        fprintf(stderr, "no LimeSDR found\n");
        fclose(wf);
        return 1;
    }
    if (LMS_Open(&dev, list[0], NULL)) {
        fprintf(stderr, "LMS_Open failed: %s\n", LMS_GetLastErrorMessage());
        fclose(wf);
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

    CHECK(LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_START));
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
    printf("TX %.6f MHz (host=%.2f Msps, rf=%.2f Msps, start_gain=%d dB (read=%u dB), target_gain=%d dB, ramp=%d ms, step=%d ms, %sconvert)\n",
           rf_hz / 1e6, host_sr / 1e6, rf_sr / 1e6, TX_GAIN_START, g_cur, TX_GAIN_DB, RAMP_MS, RAMP_INTERVAL_MS,
           NCO_DOWNCONVERT ? "down" : "up");
    printf("Streaming: %s  (Ctrl+C to stop)\n", WAV_PATH);

    buf = (int16_t *)malloc(2 * BUF_SAMPLES * sizeof(int16_t));
    if (!buf) {
        fprintf(stderr, "malloc failed\n");
        goto cleanup;
    }

    const size_t bytes_per_frame = 2 * (wi.bits_per_sample / 8);
    const size_t bytes_per_chunk = BUF_SAMPLES * bytes_per_frame;

    uint64_t bytes_left = wi.data_bytes;

    const bool use_ramp = (RAMP_MS > 0) && (TX_GAIN_DB != TX_GAIN_START);
    const int total_delta = TX_GAIN_DB - TX_GAIN_START;
    const int steps = use_ramp ? (RAMP_MS + RAMP_INTERVAL_MS - 1) / RAMP_INTERVAL_MS : 1;
    const double step_db_f = use_ramp ? ((double)total_delta / (double)steps) : 0.0;

    uint64_t t0 = now_ms();
    uint64_t t_next = t0 + (use_ramp ? (uint64_t)RAMP_INTERVAL_MS : UINT64_MAX);
    double g_accum = (double)TX_GAIN_START;
    int g_last_applied = TX_GAIN_START;

    while (keep_running) {
        size_t want = bytes_per_chunk;

        if (!LOOP) {
            if (bytes_left == 0)
                break;
            if (bytes_left < want)
                want = (size_t)bytes_left;
        }

        size_t got = fread(buf, 1, want, wf);

        if (got > 0 && SCALE != 1.0) {
            size_t samples16 = got / 2;
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

        size_t frames = got / bytes_per_frame;
        if (frames > 0) {
            lms_stream_meta_t meta;
            memset(&meta, 0, sizeof(meta));
            if (LMS_SendStream(&txs, buf, frames, &meta, SEND_TIMEOUT_MS) < 0) {
                fprintf(stderr, "LMS_SendStream error: %s\n", LMS_GetLastErrorMessage());
                break;
            }

            if (use_ramp) {
                uint64_t now = now_ms();
                while (now >= t_next && keep_running) {
                    g_accum += step_db_f;
                    int g_int = clampi((int)llround(g_accum), TX_GAIN_MIN_DB, TX_GAIN_MAX_DB);

                    if (g_int != g_last_applied) {
                        fprintf(stderr, "ramp: setting gain to %d dB\n", g_int);
                        if (LMS_SetGaindB(dev, LMS_CH_TX, CH, g_int) == 0) {
                            unsigned int g_read = 0;
                            if (LMS_GetGaindB(dev, LMS_CH_TX, CH, &g_read) == 0) {
                                fprintf(stderr, "ramp: set=%d dB, get=%u dB\n", g_int, g_read);
                            } else {
                                fprintf(stderr, "ramp: LMS_GetGaindB failed: %s\n",
                                        LMS_GetLastErrorMessage());
                            }
                            g_last_applied = g_int;
                        } else {
                            fprintf(stderr, "Gain ramp set failed: %s\n", LMS_GetLastErrorMessage());
                        }
                    }

                    t_next += (uint64_t)RAMP_INTERVAL_MS;

                    if ((step_db_f >= 0.0 && g_last_applied >= TX_GAIN_DB) ||
                        (step_db_f <  0.0 && g_last_applied <= TX_GAIN_DB)) {
                        if (g_last_applied != TX_GAIN_DB) {
                            (void)LMS_SetGaindB(dev, LMS_CH_TX, CH, TX_GAIN_DB);
                            g_last_applied = TX_GAIN_DB;
                        }
                        t_next = UINT64_MAX;
                        break;
                    }
                    now = now_ms();
                }
            }
        }

        if (!LOOP) {
            if (got < want) {
                memset(buf, 0, 2 * BUF_SAMPLES * sizeof(int16_t));
                lms_stream_meta_t meta;
                memset(&meta, 0, sizeof(meta));
                (void)LMS_SendStream(&txs, buf, BUF_SAMPLES, &meta, SEND_TIMEOUT_MS);
                break;
            }
            bytes_left -= got;
        } else {
            if (got < want) {
                fseek(wf, (long)wi.data_offset, SEEK_SET);
                bytes_left = wi.data_bytes;
            }
        }
    }

    printf("\nSIGINT or EOF, stopping\n");

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
        printf("TX channel disabled\n");
        LMS_Close(dev);
    }

    if (wf)
        fclose(wf);
    if (buf)
        free(buf);
    return 0;
}
