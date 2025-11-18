#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import signal
import time
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_CS16, errToStr

# === Константы "как в C" ===
CH = 0
NCO_INDEX = 0            # в Soapy напрямую не используется
FIFO_SIZE_SAMPLES = 1 << 17
BUF_SAMPLES = 8192
SEND_TIMEOUT_MS = 1000
TONE_SCALE = 0.70

keep_running = True


def on_sigint(signum, frame):
    global keep_running
    keep_running = False


# --- парсинг аргументов как в оригинале ---

def parse_bool(s):
    if s is None:
        return None
    s = s.strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off"):
        return False
    return None


def parse_hz(s):
    """
    Строка вида: "5e6", "5M", "10 k", "2.5G" и т.п.
    Возвращает float (Гц) или None при ошибке.
    """
    import re
    if s is None:
        return None
    s = s.strip()
    m = re.match(
        r'^([+-]?(?:\d+\.?\d*|\d*\.?\d+)(?:[eE][+-]?\d+)?)\s*([kKmMgG]?)\s*$',
        s
    )
    if not m:
        return None
    v = float(m.group(1))
    suffix = m.group(2).lower()
    if suffix == 'k':
        v *= 1e3
    elif suffix == 'm':
        v *= 1e6
    elif suffix == 'g':
        v *= 1e9
    return v


def parse_args(argv):
    # значения по умолчанию как в C
    HOST_SR_HZ      = 5e6
    OVERSAMPLE      = 32
    TX_LPF_BW_HZ    = 20e6
    LO_HZ           = 30e6
    NCO_FREQ_HZ     = 15e6
    NCO_DOWNCONVERT = True
    TX_GAIN_DB      = 40
    CAL_BW_HZ       = -1.0

    i = 1
    while i < len(argv):
        a = argv[i]

        def needval():
            if i + 1 >= len(argv):
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
            b = parse_bool(argv[i])
            if b is None:
                print("bad --nco-downconvert", file=sys.stderr)
                sys.exit(1)
            NCO_DOWNCONVERT = b

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
        CAL_BW_HZ = TX_LPF_BW_HZ

    return (HOST_SR_HZ, OVERSAMPLE, TX_LPF_BW_HZ,
            LO_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT, TX_GAIN_DB, CAL_BW_HZ)


def main():
    global keep_running

    (HOST_SR_HZ, OVERSAMPLE, TX_LPF_BW_HZ,
     LO_HZ, NCO_FREQ_HZ, NCO_DOWNCONVERT, TX_GAIN_DB, CAL_BW_HZ) = parse_args(sys.argv)

    signal.signal(signal.SIGINT, on_sigint)

    # --- поиск устройства (как LMS_GetDeviceList + LMS_Open) ---
    results = SoapySDR.Device.enumerate("driver=lime")
    if not results:
        results = SoapySDR.Device.enumerate()
    if not results:
        print("no LimeSDR (SoapySDR) devices found", file=sys.stderr)
        sys.exit(1)

    print(f"using device: {results[0]}")
    sdr = SoapySDR.Device(results[0])

    # --- включаем TX канал (аналог LMS_EnableChannel TX CH true) ---
    # В Soapy отдельного EnableChannel нет, но setSampleRate/setFrequency/stream
    # фактически "включают" тракт.

    # Установка частоты дискретизации
    sdr.setSampleRate(SOAPY_SDR_TX, CH, HOST_SR_HZ)

    # OVERSAMPLING (если поддерживается драйвером)
    if OVERSAMPLE != 1:
        try:
            sdr.writeSetting(SOAPY_SDR_TX, CH, "OVERSAMPLING", str(OVERSAMPLE))
        except Exception as e:
            print(f"warning: OVERSAMPLING not supported by driver: {e}",
                  file=sys.stderr)

    host_sr = sdr.getSampleRate(SOAPY_SDR_TX, CH)
    rf_sr = host_sr * OVERSAMPLE
    print(f"set/get: sample rate host={host_sr/1e6:.2f} Msps, rf={rf_sr/1e6:.2f} Msps")

    # Полоса TX LPF
    sdr.setBandwidth(SOAPY_SDR_TX, CH, TX_LPF_BW_HZ)

    # Усиление
    sdr.setGain(SOAPY_SDR_TX, CH, TX_GAIN_DB)
    g_cur = sdr.getGain(SOAPY_SDR_TX, CH)
    print(f"set/get: TX gain = {g_cur:.0f} dB")

    # LO (RF частота)
    sdr.setFrequency(SOAPY_SDR_TX, CH, "RF", LO_HZ)
    lof = sdr.getFrequency(SOAPY_SDR_TX, CH, "RF")
    print(f"set/get: LO freq = {lof/1e6:.6f} MHz")

    # NCO: в Soapy делаем это через BB-частоту
    bb_freq = -NCO_FREQ_HZ if NCO_DOWNCONVERT else NCO_FREQ_HZ
    sdr.setFrequency(SOAPY_SDR_TX, CH, "BB", bb_freq)
    bb_read = sdr.getFrequency(SOAPY_SDR_TX, CH, "BB")
    print(f"set/get: NCO (BB) freq = {bb_read/1e6:.6f} MHz "
          f"({'down' if NCO_DOWNCONVERT else 'up'}convert)")

    # # Аналога LMS_Calibrate в Soapy-API нет, калибровка обычно происходит
    # # автоматически в драйвере Lime. Здесь просто выведем сообщение.
    # print(f"TX calibrated (logical bw={CAL_BW_HZ/1e6:.2f} MHz) [Soapy driver]")

    # --- Настройка и запуск TX-стрима (аналог LMS_SetupStream + StartStream) ---
    tx_stream = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [CH])
    sdr.activateStream(tx_stream)
    print(f"TX stream started (driver FIFO is internal, BUF={BUF_SAMPLES} samples, fmt=CS16)")

    # --- Буфер с постоянным тоном: I = const, Q = 0 ---
    I = int(round(TONE_SCALE * 32767.0))
    Q = 0
    buf = np.empty(2 * BUF_SAMPLES, dtype=np.int16)
    buf[0::2] = I
    buf[1::2] = Q

    rf_hz = LO_HZ + bb_freq
    print("TX %.6f MHz (host=%.2f Msps, rf=%.2f Msps, gain=%.0f dB, %sconvert)" %
          (rf_hz / 1e6, host_sr / 1e6, rf_sr / 1e6, g_cur,
           "down" if NCO_DOWNCONVERT else "up"))
    print("Ctrl+C to stop")

    # --- основной цикл передачи ---
    try:
        while keep_running:
            rc = sdr.writeStream(
                tx_stream,
                [buf],
                BUF_SAMPLES,
                timeoutUs=SEND_TIMEOUT_MS * 1000
            )
            if rc.ret < 0:
                print(f"LMS_SendStream error (Soapy writeStream): {errToStr(rc.ret)}",
                      file=sys.stderr)
                break
            # можно чуть притормозить, чтобы не долбить CPU
            # time.sleep(0.0)
    except KeyboardInterrupt:
        keep_running = False

    print("\nSIGINT detected")

    # --- очистка (аналог cleanup:) ---
    try:
        # "сброс" цепочки нулями, как в C-версии
        zero_buf = np.zeros_like(buf)
        sdr.writeStream(
            tx_stream,
            [zero_buf],
            BUF_SAMPLES,
            timeoutUs=SEND_TIMEOUT_MS * 1000
        )
    except Exception:
        pass

    sdr.deactivateStream(tx_stream)
    sdr.closeStream(tx_stream)
    print("TX stream stopped")

    # В Soapy нет явного EnableChannel(false), просто удаляем объект
    del sdr

    return 0


if __name__ == "__main__":
    sys.exit(main())
