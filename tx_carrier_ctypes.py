#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import ctypes
import math
import signal
import sys

# -----------------------------
# Константы из исходника C
# -----------------------------

CH = 0
NCO_INDEX = 0
FIFO_SIZE_SAMPLES = 1 << 17
BUF_SAMPLES = 8192
SEND_TIMEOUT_MS = 1000
TONE_SCALE = 0.70

# Значения каналов и форматов – проверьте по LimeSuite.h
LMS_CH_RX = 0
LMS_CH_TX = 1

# Предполагаемые значения enum lms_stream_t::LMS_FMT_*
# ОБЯЗАТЕЛЬНО сверить с LimeSuite.h!
LMS_FMT_F32 = 0
LMS_FMT_I16 = 1
LMS_FMT_I12 = 2

# -----------------------------
# Загрузка libLimeSuite
# -----------------------------

def load_lime_library():
    cand = [
        "libLimeSuite.so",
        "libLimeSuite.dll",
        "LimeSuite.dll",
        "libLimeSuite.dylib",
    ]
    last_err = None
    for name in cand:
        try:
            return ctypes.CDLL(name)
        except OSError as e:
            last_err = e
    raise RuntimeError(f"Не удалось загрузить LimeSuite: {last_err}")

lime = load_lime_library()

# -----------------------------
# Типы и структуры
# -----------------------------

# В C: typedef char lms_info_str_t[256];
class lms_info_str_t(ctypes.Array):
    _type_ = ctypes.c_char
    _length_ = 256

# В C: typedef struct lms_device_struct lms_device_t; (opaque)
lms_device_t_p = ctypes.c_void_p

# !!! ВАЖНО !!!
# Определения ниже взяты по косвенным источникам и здравому смыслу.
# Обязательно откройте у себя LimeSuite.h и поправьте порядок/типы полей.
# Здесь главное — показать, как это делается в ctypes.

class lms_stream_t(ctypes.Structure):
    _fields_ = [
        # ПОРЯДОК И ТИПЫ ПРОВЕРЯЙТЕ ПО LimeSuite.h!
        ("handle", ctypes.c_void_p),          # внутренний указатель (обычно)
        ("isTx", ctypes.c_bool),              # направление: True=TX, False=RX
        ("channel", ctypes.c_uint32),         # номер канала
        ("fifoSize", ctypes.c_uint32),        # размер FIFO в семплах
        ("throughputVsLatency", ctypes.c_double),  # 0..1
        ("dataFmt", ctypes.c_int),            # формат данных (enum)
        # небольшой запас, если в вашей версии больше полей
        ("_reserved", ctypes.c_uint8 * 40),
    ]

class lms_stream_meta_t(ctypes.Structure):
    _fields_ = [
        # снова – типы и порядок нужно сверить с вашей версией LimeSuite.h
        ("timestamp", ctypes.c_uint64),
        ("waitForTimestamp", ctypes.c_bool),
        ("flushPartialPacket", ctypes.c_bool),
        ("_reserved", ctypes.c_uint8 * 14),
    ]

# Настроим только то, что точно знаем
lime.LMS_GetLastErrorMessage.restype = ctypes.c_char_p


def last_error():
    msg = lime.LMS_GetLastErrorMessage()
    return msg.decode("utf-8", errors="ignore") if msg else "<no message>"


# -----------------------------
# Вспомогательный CHECK (аналог макроса)
# -----------------------------

class LmsError(RuntimeError):
    pass


def CHECK(code, expr: str):
    if code != 0:
        raise LmsError(f"ERROR: {expr} -> {last_error()}")


# -----------------------------
# Обработка SIGINT
# -----------------------------

keep_running = True


def on_sigint(signum, frame):
    global keep_running
    keep_running = False


signal.signal(signal.SIGINT, on_sigint)

# -----------------------------
# Вспомогательные функции печати
# -----------------------------

def print_sr(dev):
    host = ctypes.c_double(0.0)
    rf = ctypes.c_double(0.0)
    if lime.LMS_GetSampleRate(dev, LMS_CH_TX, CH,
                              ctypes.byref(host), ctypes.byref(rf)) == 0:
        print(f"set/get: sample rate host={host.value/1e6:.2f} Msps, "
              f"rf={rf.value/1e6:.2f} Msps")


def print_gain(dev):
    g = ctypes.c_uint()
    if lime.LMS_GetGaindB(dev, LMS_CH_TX, CH, ctypes.byref(g)) == 0:
        print(f"set/get: TX gain = {g.value} dB")


def print_lo(dev):
    lof = ctypes.c_double(0.0)
    if lime.LMS_GetLOFrequency(dev, LMS_CH_TX, CH, ctypes.byref(lof)) == 0:
        print(f"set/get: LO freq = {lof.value/1e6:.6f} MHz")


def print_nco(dev):
    idx = lime.LMS_GetNCOIndex(dev, True, CH)
    print(f"set/get: NCO idx={idx} "
          f"(no frequency readback in this LimeSuite)")


# -----------------------------
# Парсинг аргументов
# -----------------------------

def parse_bool_str(s: str) -> bool:
    if s is None:
        raise argparse.ArgumentTypeError("bool string is None")
    v = s.strip().lower()
    if v in ("1", "true", "yes", "on"):
        return True
    if v in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"bad boolean value: {s!r}")


def parse_hz(s: str) -> float:
    """
    Разбор значений вида:
      5e6, 5M, 2.5 k, 10G и т.п.
    """
    if s is None:
        raise argparse.ArgumentTypeError("empty string")
    s = s.strip()
    if not s:
        raise argparse.ArgumentTypeError("empty string")
    # сначала число
    num_part = ""
    i = 0
    while i < len(s) and (s[i].isdigit() or s[i] in "+-.eE"):
        num_part += s[i]
        i += 1
    if not num_part:
        raise argparse.ArgumentTypeError(f"bad frequency: {s!r}")
    try:
        v = float(num_part)
    except ValueError:
        raise argparse.ArgumentTypeError(f"bad frequency: {s!r}")
    rest = s[i:].strip()
    if rest:
        c = rest[0].lower()
        if c == "k":
            mul = 1e3
        elif c == "m":
            mul = 1e6
        elif c == "g":
            mul = 1e9
        else:
            raise argparse.ArgumentTypeError(f"bad suffix in {s!r}")
        if rest[1:].strip():
            raise argparse.ArgumentTypeError(f"extra chars in {s!r}")
        v *= mul
    return v


def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Простой пример TX на LimeSuite через ctypes",
        add_help=True,
    )
    p.add_argument("--host-sr", type=parse_hz, default=5e6,
                   help="Host sample rate (Hz, можно с k/M/G)")
    p.add_argument("--oversample", type=int, default=32,
                   help="RF oversampling (integer >=1)")
    p.add_argument("--tx-lpf-bw", type=parse_hz, default=20e6,
                   help="TX LPF bandwidth (Hz)")
    p.add_argument("--lo", type=parse_hz, default=30e6,
                   help="LO frequency (Hz)")
    p.add_argument("--nco", type=parse_hz, default=15e6,
                   help="NCO frequency (Hz)")
    p.add_argument("--nco-downconvert", type=parse_bool_str, default=True,
                   help="true/false – NCO downconvert")
    p.add_argument("--tx-gain", type=int, default=40,
                   help="TX gain in dB (0..73 typical)")
    p.add_argument("--cal-bw", type=parse_hz, default=-1.0,
                   help="Calibration bandwidth (Hz), <=0 => tx-lpf-bw")
    return p


# -----------------------------
# Основная логика
# -----------------------------

def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    HOST_SR_HZ = args.host_sr
    OVERSAMPLE = args.oversample
    TX_LPF_BW_HZ = args.tx_lpf_bw
    LO_HZ = args.lo
    NCO_FREQ_HZ = args.nco
    NCO_DOWNCONVERT = args.nco_downconvert
    TX_GAIN_DB = args.tx_gain
    CAL_BW_HZ = args.cal_bw

    if OVERSAMPLE < 1:
        parser.error("bad --oversample (must be >=1)")
    if TX_GAIN_DB < 0 or TX_GAIN_DB > 73:
        parser.error("--tx-gain (must be 0..73 dB typical)")

    if CAL_BW_HZ <= 0:
        CAL_BW_HZ = TX_LPF_BW_HZ

    dev = lms_device_t_p()
    txs = lms_stream_t()
    stream_started = False
    buf = None

    try:
        # -----------------------------
        # Поиск и открытие устройства
        # -----------------------------
        lst_type = lms_info_str_t * 8
        dev_list = lst_type()

        # Если передать NULL, можно получить только количество.
        # Но нам всё равно нужен список, как в вашем коде.
        n = lime.LMS_GetDeviceList(dev_list)
        if n < 1:
            print("no LimeSDR found", file=sys.stderr)
            return 1

        # Открываем первый девайс
        # int LMS_Open(lms_device_t **device, const lms_info_str_t info, void *args);
        dev_ptr = ctypes.pointer(dev)
        CHECK(lime.LMS_Open(dev_ptr, dev_list[0], None), "LMS_Open")
        dev = dev_ptr.contents

        # -----------------------------
        # Инициализация и базовая настройка
        # -----------------------------
        CHECK(lime.LMS_Init(dev), "LMS_Init")

        # CHECK(lime.LMS_Reset(dev), "LMS_Reset")
        # print("device reset to defaults")

        CHECK(lime.LMS_EnableChannel(dev, LMS_CH_TX, CH, True),
              "LMS_EnableChannel(TX)")
        print("TX channel enabled")

        CHECK(lime.LMS_SetSampleRate(dev, ctypes.c_double(HOST_SR_HZ),
                                     ctypes.c_uint(OVERSAMPLE)),
              "LMS_SetSampleRate")
        print_sr(dev)

        CHECK(lime.LMS_SetLPFBW(dev, LMS_CH_TX, CH,
                                ctypes.c_double(TX_LPF_BW_HZ)),
              "LMS_SetLPFBW")

        CHECK(lime.LMS_SetGaindB(dev, LMS_CH_TX, CH,
                                 ctypes.c_uint(TX_GAIN_DB)),
              "LMS_SetGaindB")
        print_gain(dev)

        CHECK(lime.LMS_SetLOFrequency(dev, LMS_CH_TX, CH,
                                      ctypes.c_double(LO_HZ)),
              "LMS_SetLOFrequency")
        print_lo(dev)

        # -----------------------------
        # NCO настройка
        # -----------------------------
        freqs = (ctypes.c_double * 16)()
        freqs[NCO_INDEX] = ctypes.c_double(NCO_FREQ_HZ)
        CHECK(lime.LMS_SetNCOFrequency(dev, True, CH,
                                       freqs, ctypes.c_double(0.0)),
              "LMS_SetNCOFrequency")

        CHECK(lime.LMS_SetNCOIndex(dev, True, CH,
                                   ctypes.c_int(NCO_INDEX),
                                   ctypes.c_bool(NCO_DOWNCONVERT)),
              "LMS_SetNCOIndex")

        idx = lime.LMS_GetNCOIndex(dev, True, CH)
        if idx < 0:
            raise LmsError(f"LMS_GetNCOIndex failed: {last_error()}")
        print_nco(dev)

        # -----------------------------
        # Калибровка TX
        # -----------------------------
        CHECK(lime.LMS_Calibrate(dev, LMS_CH_TX, CH,
                                 ctypes.c_double(CAL_BW_HZ),
                                 ctypes.c_uint(0)),
              "LMS_Calibrate")
        print(f"TX calibrated (bw={CAL_BW_HZ/1e6:.2f} MHz)")

        # -----------------------------
        # Настройка TX stream
        # -----------------------------
        # memset(&txs, 0, sizeof)
        ctypes.memset(ctypes.byref(txs), 0, ctypes.sizeof(txs))
        txs.channel = CH
        txs.isTx = True
        txs.fifoSize = FIFO_SIZE_SAMPLES
        txs.dataFmt = LMS_FMT_I16
        # TX latency/throughput по желанию, здесь оставим 0 (по умолчанию)
        # txs.throughputVsLatency = 0.5

        CHECK(lime.LMS_SetupStream(dev, ctypes.byref(txs)),
              "LMS_SetupStream")
        CHECK(lime.LMS_StartStream(ctypes.byref(txs)),
              "LMS_StartStream")
        stream_started = True
        print(f"TX stream started (fifo={FIFO_SIZE_SAMPLES} samples, fmt=I16)")

        # -----------------------------
        # Заполнение буфера постоянным тоном
        # -----------------------------
        buf_type = ctypes.c_int16 * (2 * BUF_SAMPLES)
        buf = buf_type()
        I_val = int(TONE_SCALE * 32767.0)
        Q_val = 0
        for i in range(BUF_SAMPLES):
            buf[2 * i] = I_val
            buf[2 * i + 1] = Q_val

        # -----------------------------
        # Вывод текущих настроек
        # -----------------------------
        host_sr = ctypes.c_double()
        rf_sr = ctypes.c_double()
        lime.LMS_GetSampleRate(dev, LMS_CH_TX, CH,
                               ctypes.byref(host_sr), ctypes.byref(rf_sr))

        g_cur = ctypes.c_uint()
        lime.LMS_GetGaindB(dev, LMS_CH_TX, CH, ctypes.byref(g_cur))

        rf_hz = LO_HZ - NCO_FREQ_HZ if NCO_DOWNCONVERT else LO_HZ + NCO_FREQ_HZ
        print(f"TX {rf_hz/1e6:.6f} MHz "
              f"(host={host_sr.value/1e6:.2f} Msps, "
              f"rf={rf_sr.value/1e6:.2f} Msps, "
              f"gain={g_cur.value} dB, "
              f"{'down' if NCO_DOWNCONVERT else 'up'}convert)")
        print("Ctrl+C to stop")

        # -----------------------------
        # Главный цикл передачи
        # -----------------------------
        meta = lms_stream_meta_t()
        ctypes.memset(ctypes.byref(meta), 0, ctypes.sizeof(meta))

        while keep_running:
            # ssize_t LMS_SendStream(lms_stream_t *stream, const void *samples,
            #                        size_t sample_count, lms_stream_meta_t *meta,
            #                        unsigned timeout_ms);
            sent = lime.LMS_SendStream(ctypes.byref(txs), ctypes.byref(buf),
                                       ctypes.c_size_t(BUF_SAMPLES),
                                       ctypes.byref(meta),
                                       ctypes.c_uint(SEND_TIMEOUT_MS))
            if sent < 0:
                print(f"LMS_SendStream error: {last_error()}",
                      file=sys.stderr)
                break

        print("\nSIGINT detected")

    except LmsError as e:
        print(str(e), file=sys.stderr)
    finally:
        # -----------------------------
        # cleanup (аналог goto cleanup)
        # -----------------------------
        if stream_started:
            # Попробуем «заткнуть» тракт нулями
            try:
                z_type = ctypes.c_int16 * (2 * BUF_SAMPLES)
                z = z_type()
                meta2 = lms_stream_meta_t()
                ctypes.memset(ctypes.byref(meta2), 0, ctypes.sizeof(meta2))
                lime.LMS_SendStream(ctypes.byref(txs), ctypes.byref(z),
                                    ctypes.c_size_t(BUF_SAMPLES),
                                    ctypes.byref(meta2),
                                    ctypes.c_uint(SEND_TIMEOUT_MS))
            except Exception:
                pass

            try:
                lime.LMS_StopStream(ctypes.byref(txs))
                lime.LMS_DestroyStream(dev, ctypes.byref(txs))
                print("TX stream stopped")
            except Exception:
                pass

        if dev:
            try:
                lime.LMS_EnableChannel(dev, LMS_CH_TX, CH, False)
                print("TX channel disabled")
            except Exception:
                pass
            try:
                lime.LMS_Close(dev)
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
