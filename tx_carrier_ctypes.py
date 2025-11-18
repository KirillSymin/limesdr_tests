#!/usr/bin/env python3
import sys
import math
import signal
import ctypes as ct

# ------------------------------------------------------------
# Константы из C-кода
# ------------------------------------------------------------
CH = 0
NCO_INDEX = 0
FIFO_SIZE_SAMPLES = 1 << 17
BUF_SAMPLES = 8192
SEND_TIMEOUT_MS = 1000
TONE_SCALE = 0.70

LMS_SUCCESS = 0

# эквивалент LMS_CH_TX = true
LMS_CH_TX = True
LMS_CH_RX = False

# Форматы данных
LMS_FMT_F32 = 0
LMS_FMT_I16 = 1
LMS_FMT_I12 = 2

LMS_LINK_FMT_DEFAULT = 0
LMS_LINK_FMT_I16 = 1
LMS_LINK_FMT_I12 = 2

# ------------------------------------------------------------
# Загрузка библиотеки LimeSuite
# ------------------------------------------------------------
def load_limesuite():
    import os
    from ctypes.util import find_library

    # Пытаемся найти библиотеку автоматически
    libname = find_library("LimeSuite")
    candidates = []

    if libname:
        candidates.append(libname)

    if sys.platform.startswith("linux"):
        candidates += ["libLimeSuite.so"]
    elif sys.platform == "darwin":
        candidates += ["libLimeSuite.dylib", "libLimeSuite.so"]
    elif sys.platform == "win32":
        candidates += ["LimeSuite.dll"]

    last_err = None
    for name in candidates:
        try:
            return ct.CDLL(name)
        except OSError as e:
            last_err = e

    raise OSError(f"Cannot load LimeSuite library, tried: {candidates}, last error: {last_err}")


lib = load_limesuite()

# ------------------------------------------------------------
# Типы / структуры из LimeSuite.h
# ------------------------------------------------------------

# typedef double float_type;
float_type = ct.c_double

# typedef void lms_device_t;
lms_device_t_p = ct.c_void_p  # handle

# typedef char lms_info_str_t[256];
class lms_info_str_t(ct.Array):
    _type_ = ct.c_char
    _length_ = 256

# lms_stream_meta_t
class lms_stream_meta_t(ct.Structure):
    _fields_ = [
        ("timestamp", ct.c_uint64),
        ("waitForTimestamp", ct.c_bool),
        ("flushPartialPacket", ct.c_bool),
    ]

# lms_stream_t
class lms_stream_t(ct.Structure):
    _fields_ = [
        ("handle", ct.c_size_t),
        ("isTx", ct.c_bool),
        ("channel", ct.c_uint32),
        ("fifoSize", ct.c_uint32),
        ("throughputVsLatency", float_type),
        ("dataFmt", ct.c_int),
        ("linkFmt", ct.c_int),
    ]

# ------------------------------------------------------------
# Прототипы используемых функций
# ------------------------------------------------------------

# int LMS_GetDeviceList(lms_info_str_t *dev_list);
lib.LMS_GetDeviceList.restype = ct.c_int
lib.LMS_GetDeviceList.argtypes = [ct.POINTER(lms_info_str_t)]

# int LMS_Open(lms_device_t **device, const lms_info_str_t info, void* args);
lib.LMS_Open.restype = ct.c_int
lib.LMS_Open.argtypes = [ct.POINTER(lms_device_t_p), lms_info_str_t, ct.c_void_p]

# int LMS_Close(lms_device_t *device);
lib.LMS_Close.restype = ct.c_int
lib.LMS_Close.argtypes = [lms_device_t_p]

# int LMS_Init(lms_device_t *device);
lib.LMS_Init.restype = ct.c_int
lib.LMS_Init.argtypes = [lms_device_t_p]

# int LMS_EnableChannel(lms_device_t *device, bool dir_tx, size_t chan, bool enabled);
lib.LMS_EnableChannel.restype = ct.c_int
lib.LMS_EnableChannel.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t, ct.c_bool]

# int LMS_SetSampleRate(lms_device_t *device, float_type rate, size_t oversample);
lib.LMS_SetSampleRate.restype = ct.c_int
lib.LMS_SetSampleRate.argtypes = [lms_device_t_p, float_type, ct.c_size_t]

# int LMS_GetSampleRate(lms_device_t *device, bool dir_tx, size_t chan, float_type *host_Hz, float_type *rf_Hz);
lib.LMS_GetSampleRate.restype = ct.c_int
lib.LMS_GetSampleRate.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t,
                                  ct.POINTER(float_type), ct.POINTER(float_type)]

# int LMS_SetLPFBW(lms_device_t *device, bool dir_tx, size_t chan, float_type bandwidth);
lib.LMS_SetLPFBW.restype = ct.c_int
lib.LMS_SetLPFBW.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t, float_type]

# int LMS_SetGaindB(lms_device_t *device, bool dir_tx, size_t chan, unsigned gain);
lib.LMS_SetGaindB.restype = ct.c_int
lib.LMS_SetGaindB.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t, ct.c_uint]

# int LMS_GetGaindB(lms_device_t *device, bool dir_tx, size_t chan, unsigned *gain);
lib.LMS_GetGaindB.restype = ct.c_int
lib.LMS_GetGaindB.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t, ct.POINTER(ct.c_uint)]

# int LMS_SetLOFrequency(lms_device_t *device, bool dir_tx, size_t chan, float_type frequency);
lib.LMS_SetLOFrequency.restype = ct.c_int
lib.LMS_SetLOFrequency.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t, float_type]

# int LMS_GetLOFrequency(lms_device_t *device, bool dir_tx, size_t chan, float_type *frequency);
lib.LMS_GetLOFrequency.restype = ct.c_int
lib.LMS_GetLOFrequency.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t,
                                   ct.POINTER(float_type)]

# int LMS_SetNCOFrequency(lms_device_t *device, bool dir_tx, size_t chan,
#                         const float_type *freq, float_type pho);
lib.LMS_SetNCOFrequency.restype = ct.c_int
lib.LMS_SetNCOFrequency.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t,
                                    ct.POINTER(float_type), float_type]

# int LMS_SetNCOIndex(lms_device_t *device, bool dir_tx, size_t chan, int index, bool downconv);
lib.LMS_SetNCOIndex.restype = ct.c_int
lib.LMS_SetNCOIndex.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t,
                                ct.c_int, ct.c_bool]

# int LMS_GetNCOIndex(lms_device_t *device, bool dir_tx, size_t chan);
lib.LMS_GetNCOIndex.restype = ct.c_int
lib.LMS_GetNCOIndex.argtypes = [lms_device_t_p, ct.c_bool, ct.c_size_t]

# int LMS_SetupStream(lms_device_t *device, lms_stream_t *stream);
lib.LMS_SetupStream.restype = ct.c_int
lib.LMS_SetupStream.argtypes = [lms_device_t_p, ct.POINTER(lms_stream_t)]

# int LMS_StartStream(lms_stream_t *stream);
lib.LMS_StartStream.restype = ct.c_int
lib.LMS_StartStream.argtypes = [ct.POINTER(lms_stream_t)]

# int LMS_StopStream(lms_stream_t *stream);
lib.LMS_StopStream.restype = ct.c_int
lib.LMS_StopStream.argtypes = [ct.POINTER(lms_stream_t)]

# int LMS_DestroyStream(lms_device_t *dev, lms_stream_t *stream);
lib.LMS_DestroyStream.restype = ct.c_int
lib.LMS_DestroyStream.argtypes = [lms_device_t_p, ct.POINTER(lms_stream_t)]

# int LMS_SendStream(lms_stream_t *stream, const void *samples,
#                    size_t sample_count, const lms_stream_meta_t *meta,
#                    unsigned timeout_ms);
lib.LMS_SendStream.restype = ct.c_int
lib.LMS_SendStream.argtypes = [ct.POINTER(lms_stream_t),
                               ct.c_void_p,
                               ct.c_size_t,
                               ct.POINTER(lms_stream_meta_t),
                               ct.c_uint]

# const char * LMS_GetLastErrorMessage(void);
lib.LMS_GetLastErrorMessage.restype = ct.c_char_p
lib.LMS_GetLastErrorMessage.argtypes = []


# ------------------------------------------------------------
# Вспомогательные функции – аналог CHECK(x) и print_*
# ------------------------------------------------------------
def last_error():
    msg = lib.LMS_GetLastErrorMessage()
    return msg.decode("utf-8", errors="replace") if msg else "Unknown error"


def CHECK(ret, what=""):
    if ret != LMS_SUCCESS:
        if not what:
            what = "LimeSuite call"
        raise RuntimeError(f"ERROR: {what} -> {last_error()}")


def print_sr(dev):
    host = float_type(0.0)
    rf = float_type(0.0)
    if lib.LMS_GetSampleRate(dev, LMS_CH_TX, CH, ct.byref(host), ct.byref(rf)) == 0:
        print(f"set/get: sample rate host={host.value/1e6:.2f} Msps, rf={rf.value/1e6:.2f} Msps")


def print_gain(dev):
    g = ct.c_uint(0)
    if lib.LMS_GetGaindB(dev, LMS_CH_TX, CH, ct.byref(g)) == 0:
        print(f"set/get: TX gain = {g.value} dB")


def print_lo(dev):
    lof = float_type(0.0)
    if lib.LMS_GetLOFrequency(dev, LMS_CH_TX, CH, ct.byref(lof)) == 0:
        print(f"set/get: LO freq = {lof.value/1e6:.6f} MHz")


def print_nco(dev):
    idx = lib.LMS_GetNCOIndex(dev, LMS_CH_TX, CH)
    print(f"set/get: NCO idx={idx} (no frequency readback in this LimeSuite)")


# ------------------------------------------------------------
# Парсинг значений (bool, Hz) – перенос функций из C
# ------------------------------------------------------------
def parse_bool(s: str):
    if s is None:
        return None
    s2 = s.strip().lower()
    if s2 in ("1", "true", "yes", "on"):
        return True
    if s2 in ("0", "false", "no", "off"):
        return False
    return None


def parse_hz(s: str):
    if s is None:
        return None
    s = s.strip()
    try:
        # парсим число
        # отделяем суффикс (k/m/g), если есть
        num = ""
        i = 0
        while i < len(s) and (s[i].isdigit() or s[i] in "+-.eE"):
            num += s[i]
            i += 1
        if not num:
            return None
        v = float(num)
        rest = s[i:].strip()
        if rest:
            mul = 1.0
            c = rest[0].lower()
            if c == 'k':
                mul = 1e3
            elif c == 'm':
                mul = 1e6
            elif c == 'g':
                mul = 1e9
            else:
                return None
            rest = rest[1:].strip()
            if rest:
                # лишние символы
                return None
            v *= mul
        return v
    except ValueError:
        return None


# ------------------------------------------------------------
# Глобальный флаг для SIGINT
# ------------------------------------------------------------
keep_running = True


def on_sigint(signum, frame):
    global keep_running
    keep_running = False


# ------------------------------------------------------------
# Основная логика (аналог main)
# ------------------------------------------------------------
def main(argv=None):
    if argv is None:
        argv = sys.argv

    # Значения по умолчанию как в C-коде
    HOST_SR_HZ = 5e6
    OVERSAMPLE = 32
    TX_LPF_BW_HZ = 20e6
    LO_HZ = 30e6
    NCO_FREQ_HZ = 15e6
    NCO_DOWNCONVERT = True
    TX_GAIN_DB = 40
    CAL_BW_HZ = -1.0

    # Простой разбор аргументов в стиле C-кода
    i = 1
    argc = len(argv)
    while i < argc:
        a = argv[i]

        def needval():
            if i + 1 >= argc:
                print(f"missing value for {a}", file=sys.stderr)
                sys.exit(1)

        if a == "--host-sr":
            needval()
            i += 1
            v = parse_hz(argv[i])
            if v is None:
                print("bad --host-sr", file=sys.stderr)
                sys.exit(1)
            HOST_SR_HZ = v
        elif a == "--oversample":
            needval()
            i += 1
            try:
                OVERSAMPLE = int(argv[i], 0)
            except ValueError:
                print("bad --oversample", file=sys.stderr)
                sys.exit(1)
            if OVERSAMPLE < 1:
                print("bad --oversample", file=sys.stderr)
                sys.exit(1)
        elif a == "--tx-lpf-bw":
            needval()
            i += 1
            v = parse_hz(argv[i])
            if v is None:
                print("bad --tx-lpf-bw", file=sys.stderr)
                sys.exit(1)
            TX_LPF_BW_HZ = v
        elif a == "--lo":
            needval()
            i += 1
            v = parse_hz(argv[i])
            if v is None:
                print("bad --lo", file=sys.stderr)
                sys.exit(1)
            LO_HZ = v
        elif a == "--nco":
            needval()
            i += 1
            v = parse_hz(argv[i])
            if v is None:
                print("bad --nco", file=sys.stderr)
                sys.exit(1)
            NCO_FREQ_HZ = v
        elif a == "--nco-downconvert":
            needval()
            i += 1
            v = parse_bool(argv[i])
            if v is None:
                print("bad --nco-downconvert", file=sys.stderr)
                sys.exit(1)
            NCO_DOWNCONVERT = v
        elif a == "--tx-gain":
            needval()
            i += 1
            try:
                TX_GAIN_DB = int(argv[i], 0)
            except ValueError:
                print("bad --tx-gain", file=sys.stderr)
                sys.exit(1)
            if TX_GAIN_DB < 0 or TX_GAIN_DB > 73:
                print("--tx-gain (must be 0..73 dB typical)", file=sys.stderr)
        elif a == "--cal-bw":
            needval()
            i += 1
            v = parse_hz(argv[i])
            if v is None:
                print("bad --cal-bw", file=sys.stderr)
                sys.exit(1)
            CAL_BW_HZ = v
        else:
            print(f"unknown option: {a}", file=sys.stderr)
            sys.exit(1)

        i += 1

    if CAL_BW_HZ <= 0:
        CAL_BW_HZ = TX_LPF_BW_HZ  # как в C, хотя дальше не используется

    # Подготовка к работе с устройством
    dev = lms_device_t_p()
    txs = lms_stream_t()
    txs.handle = 0  # важно для проверки в cleanup
    buf = None

    # обработчик SIGINT
    signal.signal(signal.SIGINT, on_sigint)

    try:
        # Получаем список устройств
        dev_list_type = lms_info_str_t * 8
        dev_list = dev_list_type()
        n = lib.LMS_GetDeviceList(dev_list)
        if n < 1:
            print("no LimeSDR found", file=sys.stderr)
            return 1

        # Открываем первое устройство
        CHECK(lib.LMS_Open(ct.byref(dev), dev_list[0], None), "LMS_Open")

        # Инициализация
        CHECK(lib.LMS_Init(dev), "LMS_Init")

        # Включаем TX канал
        CHECK(lib.LMS_EnableChannel(dev, LMS_CH_TX, CH, True), "LMS_EnableChannel")
        print("TX channel enabled")

        # Частота дискретизации
        CHECK(lib.LMS_SetSampleRate(dev, float_type(HOST_SR_HZ), OVERSAMPLE),
              "LMS_SetSampleRate")
        print_sr(dev)

        # Полоса LPF
        CHECK(lib.LMS_SetLPFBW(dev, LMS_CH_TX, CH, float_type(TX_LPF_BW_HZ)),
              "LMS_SetLPFBW")

        # Усиление
        CHECK(lib.LMS_SetGaindB(dev, LMS_CH_TX, CH, ct.c_uint(TX_GAIN_DB)),
              "LMS_SetGaindB")
        print_gain(dev)

        # LO
        CHECK(lib.LMS_SetLOFrequency(dev, LMS_CH_TX, CH, float_type(LO_HZ)),
              "LMS_SetLOFrequency")
        print_lo(dev)

        # NCO
        freqs_array = (float_type * 16)()
        for idx in range(16):
            freqs_array[idx] = float_type(0.0)
        freqs_array[NCO_INDEX] = float_type(NCO_FREQ_HZ)

        CHECK(lib.LMS_SetNCOFrequency(dev, LMS_CH_TX, CH, freqs_array,
                                      float_type(0.0)),
              "LMS_SetNCOFrequency")
        CHECK(lib.LMS_SetNCOIndex(dev, LMS_CH_TX, CH, NCO_INDEX,
                                  ct.c_bool(NCO_DOWNCONVERT)),
              "LMS_SetNCOIndex")
        idx = lib.LMS_GetNCOIndex(dev, LMS_CH_TX, CH)
        if idx < 0:
            raise RuntimeError(f"LMS_GetNCOIndex failed: {last_error()}")
        print_nco(dev)

        # Настройка потока
        # memset(&txs, 0, sizeof(txs));
        txs.handle = 0
        txs.isTx = True
        txs.channel = CH
        txs.fifoSize = FIFO_SIZE_SAMPLES
        txs.throughputVsLatency = float_type(0.0)
        txs.dataFmt = LMS_FMT_I16
        txs.linkFmt = LMS_LINK_FMT_DEFAULT

        CHECK(lib.LMS_SetupStream(dev, ct.byref(txs)), "LMS_SetupStream")
        CHECK(lib.LMS_StartStream(ct.byref(txs)), "LMS_StartStream")
        print(f"TX stream started (fifo={FIFO_SIZE_SAMPLES} samples, fmt=I16)")

        # Буфер с тоном
        buf_type = ct.c_int16 * (2 * BUF_SAMPLES)
        buf = buf_type()
        I = int(TONE_SCALE * 32767.0)
        Q = 0
        for i in range(BUF_SAMPLES):
            buf[2 * i + 0] = I
            buf[2 * i + 1] = Q

        host_sr = float_type(0.0)
        rf_sr = float_type(0.0)
        lib.LMS_GetSampleRate(dev, LMS_CH_TX, CH, ct.byref(host_sr), ct.byref(rf_sr))
        g_cur = ct.c_uint(0)
        lib.LMS_GetGaindB(dev, LMS_CH_TX, CH, ct.byref(g_cur))

        rf_hz = (LO_HZ - NCO_FREQ_HZ) if NCO_DOWNCONVERT else (LO_HZ + NCO_FREQ_HZ)
        print(
            f"TX {rf_hz/1e6:.6f} MHz "
            f"(host={host_sr.value/1e6:.2f} Msps, rf={rf_sr.value/1e6:.2f} Msps, "
            f"gain={g_cur.value} dB, {'down' if NCO_DOWNCONVERT else 'up'}convert)"
        )
        print("Ctrl+C to stop")

        # Основной цикл отправки
        meta = lms_stream_meta_t()
        meta.timestamp = 0
        meta.waitForTimestamp = False
        meta.flushPartialPacket = False

        global keep_running
        while keep_running:
            res = lib.LMS_SendStream(ct.byref(txs),
                                     ct.cast(buf, ct.c_void_p),
                                     BUF_SAMPLES,
                                     ct.byref(meta),
                                     SEND_TIMEOUT_MS)
            if res < 0:
                print(f"LMS_SendStream error: {last_error()}", file=sys.stderr)
                break

        print("\nSIGINT detected")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")
    except Exception as e:
        print(e, file=sys.stderr)
    finally:
        # cleanup: как в C-коде
        if txs.handle != 0:
            # отправляем нули
            z_type = ct.c_int16 * (2 * BUF_SAMPLES)
            z = z_type()
            meta = lms_stream_meta_t()
            meta.timestamp = 0
            meta.waitForTimestamp = False
            meta.flushPartialPacket = False
            try:
                lib.LMS_SendStream(ct.byref(txs),
                                   ct.cast(z, ct.c_void_p),
                                   BUF_SAMPLES,
                                   ct.byref(meta),
                                   SEND_TIMEOUT_MS)
            except Exception:
                pass
            try:
                lib.LMS_StopStream(ct.byref(txs))
            except Exception:
                pass
            try:
                lib.LMS_DestroyStream(dev, ct.byref(txs))
            except Exception:
                pass
            print("TX stream stopped")

        if dev:
            try:
                lib.LMS_EnableChannel(dev, LMS_CH_TX, CH, False)
                print("TX channel disabled")
            except Exception:
                pass
            try:
                lib.LMS_Close(dev)
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
