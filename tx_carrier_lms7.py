import sys
import signal

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_CS16, errToStr

CH = 0
BUF_SAMPLES = 8192
TONE_SCALE = 0.70
SEND_TIMEOUT_US = 1_000_000  # 1 s

keep_running = True


def on_sigint(signum, frame):
    global keep_running
    keep_running = False


signal.signal(signal.SIGINT, on_sigint)


def parse_bool(s: str):
    if s is None:
        return None
    s_l = s.strip().lower()
    if s_l in ("1", "true", "yes", "on"):
        return True
    if s_l in ("0", "false", "no", "off"):
        return False
    return None


def parse_hz(s: str):
    """
    Разбор значений типа: 5M, 2.5e6, 10k, 1G и т.п.
    Поведение примерно как в C-версии.
    """
    import re

    if s is None:
        return None
    m = re.match(
        r'^\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*([kKmMgG]?)\s*$',
        s,
    )
    if not m:
        return None
    val = float(m.group(1))
    suffix = m.group(2).lower()
    if suffix == "k":
        val *= 1e3
    elif suffix == "m":
        val *= 1e6
    elif suffix == "g":
        val *= 1e9
    return val


def main():
    global keep_running

    # Те же дефолты, что и в C
    HOST_SR_HZ      = 5e6
    OVERSAMPLE      = 32
    TX_LPF_BW_HZ    = 20e6
    LO_HZ           = 30e6
    NCO_FREQ_HZ     = 15e6
    NCO_DOWNCONVERT = True
    TX_GAIN_DB      = 40
    CAL_BW_HZ       = -1.0

    # Парсим argv "в стиле C"
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]

        def needval():
            if i + 1 >= len(args):
                print(f"missing value for {a}", file=sys.stderr)
                sys.exit(1)

        if a == "--host-sr":
            needval(); i += 1
            v = parse_hz(args[i])
            if v is None:
                print("bad --host-sr", file=sys.stderr)
                sys.exit(1)
            HOST_SR_HZ = v

        elif a == "--oversample":
            needval(); i += 1
            try:
                OVERSAMPLE = int(args[i], 0)
            except ValueError:
                print("bad --oversample", file=sys.stderr)
                sys.exit(1)
            if OVERSAMPLE < 1:
                print("bad --oversample", file=sys.stderr)
                sys.exit(1)

        elif a == "--tx-lpf-bw":
            needval(); i += 1
            v = parse_hz(args[i])
            if v is None:
                print("bad --tx-lpf-bw", file=sys.stderr)
                sys.exit(1)
            TX_LPF_BW_HZ = v

        elif a == "--lo":
            needval(); i += 1
            v = parse_hz(args[i])
            if v is None:
                print("bad --lo", file=sys.stderr)
                sys.exit(1)
            LO_HZ = v

        elif a == "--nco":
            needval(); i += 1
            v = parse_hz(args[i])
            if v is None:
                print("bad --nco", file=sys.stderr)
                sys.exit(1)
            NCO_FREQ_HZ = v

        elif a == "--nco-downconvert":
            needval(); i += 1
            v = parse_bool(args[i])
            if v is None:
                print("bad --nco-downconvert", file=sys.stderr)
                sys.exit(1)
            NCO_DOWNCONVERT = v

        elif a == "--tx-gain":
            needval(); i += 1
            try:
                TX_GAIN_DB = int(args[i], 0)
            except ValueError:
                print("bad --tx-gain", file=sys.stderr)
                sys.exit(1)
            if TX_GAIN_DB < 0 or TX_GAIN_DB > 73:
                print("--tx-gain (must be 0..73 dB typical)", file=sys.stderr)

        elif a == "--cal-bw":
            needval(); i += 1
            v = parse_hz(args[i])
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

    # Ищем LimeSDR через SoapyLMS7 (driver=lime)
    devs = SoapySDR.Device.enumerate("driver=lime")
    if not devs:
        print("no LimeSDR (driver=lime) found", file=sys.stderr)
        sys.exit(1)

    sdr = None
    tx_stream = None
    tx_buf = None

    try:
        sdr = SoapySDR.Device(devs[0])

        # Sample rate на хосте
        sdr.setSampleRate(SOAPY_SDR_TX, CH, HOST_SR_HZ)

        # Нестандартное расширение SoapyLMS7: oversampling.
        # Может не быть в твоей сборке, поэтому завёрнуто в try.
        try:
            sdr.writeSetting("OVERSAMPLING", str(OVERSAMPLE))
        except Exception:
            print('set OVERSAMPLING error')

        host_sr = sdr.getSampleRate(SOAPY_SDR_TX, CH)
        print(
            f"set/get: sample rate host={host_sr/1e6:.2f} Msps, "
            f"rf≈{host_sr*OVERSAMPLE/1e6:.2f} Msps"
        )

        # TX LPF
        sdr.setBandwidth(SOAPY_SDR_TX, CH, TX_LPF_BW_HZ)

        # Усиление (суммарное)
        sdr.setGain(SOAPY_SDR_TX, CH, float(TX_GAIN_DB))
        g_cur = sdr.getGain(SOAPY_SDR_TX, CH)
        print(f"set/get: TX gain = {g_cur:.1f} dB")

        # LO + NCO через компоненты частоты RF/BB в SoapyLMS7
        # RF = PLL (LO), BB = NCO offset. 
        sdr.setFrequency(SOAPY_SDR_TX, CH, "RF", LO_HZ)

        # BB < 0 → downconvert, BB > 0 → upconvert
        bb = -NCO_FREQ_HZ if NCO_DOWNCONVERT else +NCO_FREQ_HZ
        sdr.setFrequency(SOAPY_SDR_TX, CH, "BB", bb)

        lo_read = sdr.getFrequency(SOAPY_SDR_TX, CH, "RF")
        bb_read = sdr.getFrequency(SOAPY_SDR_TX, CH, "BB")
        print(f"set/get: LO freq = {lo_read/1e6:.6f} MHz")
        print(
            f"set/get: NCO (BB) freq = {bb_read/1e6:.6f} MHz, "
            f"mode={'down' if NCO_DOWNCONVERT else 'up'}convert"
        )

        # Настраиваем TX-стрим
        tx_stream = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [CH])
        sdr.activateStream(tx_stream)
        print("TX stream started (fmt=CS16)")

        # Буфер: I = const, Q = 0 (как в C-версии, int16 interleaved)
        I = int(TONE_SCALE * 32767.0)
        Q = 0
        tx_buf = np.empty(2 * BUF_SAMPLES, dtype=np.int16)
        tx_buf[0::2] = I
        tx_buf[1::2] = Q

        rf_hz = (LO_HZ - NCO_FREQ_HZ) if NCO_DOWNCONVERT else (LO_HZ + NCO_FREQ_HZ)
        print(
            f"TX {rf_hz/1e6:.6f} MHz "
            f"(host={host_sr/1e6:.2f} Msps, "
            f"rf≈{host_sr*OVERSAMPLE/1e6:.2f} Msps, "
            f"gain={g_cur:.1f} dB, "
            f"{'down' if NCO_DOWNCONVERT else 'up'}convert)"
        )
        print("Ctrl+C to stop")

        # Основной TX-цикл
        while keep_running:
            rc = sdr.writeStream(tx_stream, [tx_buf], BUF_SAMPLES,
                                 timeoutUs=SEND_TIMEOUT_US)
            if rc.ret != BUF_SAMPLES:
                print(f"writeStream error {rc.ret}: {errToStr(rc.ret)}",
                      file=sys.stderr)
                break

        print("\nSIGINT detected, stopping...")

        # «Протолкнуть» нули перед остановкой (как в C-коде)
        if tx_stream is not None:
            z = np.zeros_like(tx_buf, dtype=np.int16)
            try:
                sdr.writeStream(tx_stream, [z], BUF_SAMPLES,
                                timeoutUs=SEND_TIMEOUT_US)
            except Exception:
                pass

    finally:
        if tx_stream is not None and sdr is not None:
            try:
                sdr.deactivateStream(tx_stream)
            except Exception:
                pass
            sdr.closeStream(tx_stream)
            print("TX stream stopped")


if __name__ == "__main__":
    main()
