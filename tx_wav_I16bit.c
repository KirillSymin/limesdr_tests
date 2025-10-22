#include "lime/LimeSuite.h"
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
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

int main(int argc, char **argv) {
    int OVERSAMPLE = 32;
    double TX_LPF_BW_HZ = 20e6;
    double LO_HZ = 30e6;
    double NCO_FREQ_HZ = 15e6;
    bool NCO_DOWNCONVERT = true;
    int TX_GAIN_DB = 40;
    double CAL_BW_HZ = -1;

    const char *WAV_PATH = NULL;
    bool LOOP = false;
    double SCALE = 1.0;

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
        if (!strcmp(a,"--tx-gain")){ NEEDVAL(); TX_GAIN_DB = (int)strtol(argv[++i], NULL, 0); if (TX_GAIN_DB<0 || TX_GAIN_DB>73){ fprintf(stderr,"--tx-gain (must be 0..73 dB typical)\n"); } continue; }
        if (!strcmp(a,"--cal-bw")){ NEEDVAL(); if(!parse_hz(argv[++i], &CAL_BW_HZ)) { fprintf(stderr,"bad --cal-bw\n"); return 1; } continue; }
        if (!strcmp(a,"--loop")){ LOOP = true; continue; }
        if (!strcmp(a,"--scale")){ NEEDVAL(); SCALE = strtod(argv[++i], NULL); if (SCALE<0.0 || SCALE>4.0){ fprintf(stderr,"--scale out of range\n"); return 1; } continue; }

        fprintf(stderr,"unknown option: %s\n", a);
        return 1;
    }
    if (!WAV_PATH){ fprintf(stderr,"missing --file <path.wav>\n"); return 1; }
    if (CAL_BW_HZ <= 0) CAL_BW_HZ = TX_LPF_BW_HZ;
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

    CHECK(LMS_Init(dev));

    CHECK(LMS_Reset(dev));
    printf("device reset to defaults\n");

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

    CHECK(LMS_Calibrate(dev, LMS_CH_TX, CH, CAL_BW_HZ, 0));
    printf("TX calibrated (bw=%.2f MHz)\n", CAL_BW_HZ / 1e6);

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
    printf("TX %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%u dB, %sconvert)\n", rf_hz / 1e6, host_sr / 1e6,
           rf_sr / 1e6, g_cur, NCO_DOWNCONVERT ? "down" : "up");
    printf("Streaming: %s  (Ctrl+C to stop)\n", WAV_PATH);

    buf = (int16_t *)malloc(2 * BUF_SAMPLES * sizeof(int16_t));
    if (!buf) {
        fprintf(stderr, "malloc failed\n");
        goto cleanup;
    }

    const size_t bytes_per_frame = 2 * (wi.bits_per_sample / 8);
    const size_t bytes_per_chunk = BUF_SAMPLES * bytes_per_frame;

    uint64_t bytes_left = wi.data_bytes;
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
