#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import sys
import time
from math import isfinite

import numpy as np

# You need the LimeSuite Python bindings available as `LimeSuite`
# On most systems they come with LimeSuite installs; the module is often named `LimeSuite`.
# e.g. on Ubuntu from source packages, or via your distro's LimeSuite build.
import LimeSuite as lms  # SWIG bindings

# -----------------------------
# constants (mirroring the C)
# -----------------------------
CH                = 0           # TX channel A
HOST_SR_HZ        = 5_000_000   # 5 Msps over USB
OVERSAMPLE        = 8           # RF ≈ HOST_SR * OVERSAMPLE = 40 Msps
TX_LPF_BW_HZ      = 50_000_000  # 50 MHz LPF
LO_HZ             = 30_000_000  # 30 MHz LO (PLL-friendly)
NCO_FREQ_HZ       = 15_000_000  # 15 MHz NCO magnitude
NCO_INDEX         = 0
NCO_DOWNCONVERT   = True        # RF = LO - NCO = 15 MHz
TX_GAIN_DB        = 40          # moderate TX gain
FIFO_SIZE_SAMPLES = 1 << 17     # bigger FIFO for stability
BUF_SAMPLES       = 8192        # larger chunk reduces IRQ/USB churn
SEND_TIMEOUT_MS   = 1000
TONE_SCALE        = 0.70        # 70% FS to avoid DAC clipping

# -----------------------------
# helpers
# -----------------------------

def _errstr():
    try:
        return lms.LMS_GetLastErrorMessage().decode("utf-8", "ignore")
    except Exception:
        return "<unknown LimeSuite error>"

def CHECK(code):
    if code != 0:
        raise RuntimeError(_errstr())

def print_sr(dev):
    host = lms.doubleArray(1)
    rf = lms.doubleArray(1)
    if lms.LMS_GetSampleRate(dev, lms.LMS_CH_TX, CH, host, rf) == 0:
        print(f"Set/Get: SampleRate host={host[0]/1e6:.2f} Msps, rf={rf[0]/1e6:.2f} Msps")

def print_gain(dev):
    g = lms.uint32Array(1)
    if lms.LMS_GetGaindB(dev, lms.LMS_CH_TX, CH, g) == 0:
        print(f"Set/Get: TX Gain = {int(g[0])} dB")

def print_lo(dev):
    lof = lms.doubleArray(1)
    if lms.LMS_GetLOFrequency(dev, lms.LMS_CH_TX, CH, lof) == 0:
        print(f"Set/Get: LO = {lof[0]/1e6:.6f} MHz")

def print_nco(dev):
    idx = lms.LMS_GetNCOIndex(dev, True, CH)
    print(f"Set/Get: NCO idx={idx} (no frequency readback in this LimeSuite)")

keep_running = True
def _on_sigint(_sig, _frm):
    global keep_running
    keep_running = False

# -----------------------------
# main
# -----------------------------

def main():
    signal.signal(signal.SIGINT, _on_sigint)

    dev = None
    txs = lms.lms_stream_t()  # will populate later
    buf = None

    try:
        # 1) Open device
        lst = (lms.lms_info_str_t * 8)()
        n = lms.LMS_GetDeviceList(lst)
        if n < 1:
            print("No LimeSDR found", file=sys.stderr)
            return 1

        # pick the first device
        devp = lms.lms_device_t_p()
        if lms.LMS_Open(devp, lst[0], None) != 0:
            print(f"LMS_Open failed: {_errstr()}", file=sys.stderr)
            return 1
        dev = devp.value

        # 2) Basic setup
        CHECK(lms.LMS_Init(dev))
        CHECK(lms.LMS_EnableChannel(dev, lms.LMS_CH_TX, CH, True))
        print("TX channel enabled.")

        # Sample rates: host 5 Msps, RF ≈ 40 Msps
        CHECK(lms.LMS_SetSampleRate(dev, HOST_SR_HZ, OVERSAMPLE))
        print_sr(dev)

        # TX LPF/BW
        CHECK(lms.LMS_SetLPFBW(dev, lms.LMS_CH_TX, CH, TX_LPF_BW_HZ))

        # Gain
        CHECK(lms.LMS_SetGaindB(dev, lms.LMS_CH_TX, CH, TX_GAIN_DB))
        print_gain(dev)

        # LO
        CHECK(lms.LMS_SetLOFrequency(dev, lms.LMS_CH_TX, CH, float(LO_HZ)))
        print_lo(dev)

        # Optional calibration (commented out to match your C)
        # CHECK(lms.LMS_Calibrate(dev, lms.LMS_CH_TX, CH, 20_000_000, 0))

        # 3) NCO: program 15 MHz and select downconvert so RF = 30 - 15 = 15 MHz
        # Prepare 16-entry frequency table
        freqs = lms.doubleArray(16)
        for i in range(16):
            freqs[i] = 0.0
        freqs[NCO_INDEX] = float(NCO_FREQ_HZ)
        CHECK(lms.LMS_SetNCOFrequency(dev, True, CH, freqs, 0.0))  # dir_tx=True
        CHECK(lms.LMS_SetNCOIndex(dev, True, CH, NCO_INDEX, NCO_DOWNCONVERT))
        idx = lms.LMS_GetNCOIndex(dev, True, CH)
        if idx < 0:
            raise RuntimeError(_errstr())
        print_nco(dev)

        # 4) TX stream
        # zero-init the struct then set fields
        txs.channel = CH
        txs.isTx = True
        txs.fifoSize = FIFO_SIZE_SAMPLES
        txs.dataFmt = lms.LMS_FMT_I16  # C-safe 16-bit interleaved IQ
        CHECK(lms.LMS_SetupStream(dev, txs))
        CHECK(lms.LMS_StartStream(txs))
        print(f"TX stream started (fifo={FIFO_SIZE_SAMPLES} samples, fmt=I16).")

        # Constant DC IQ -> pure RF tone after NCO
        I = int(TONE_SCALE * 32767.0)
        Q = 0
        # interleaved IQ int16 buffer
        buf = np.empty(2 * BUF_SAMPLES, dtype=np.int16)
        buf[0::2] = I
        buf[1::2] = Q

        host_sr = lms.doubleArray(1)
        rf_sr = lms.doubleArray(1)
        lms.LMS_GetSampleRate(dev, lms.LMS_CH_TX, CH, host_sr, rf_sr)  # best-effort

        g_cur = lms.uint32Array(1)
        lms.LMS_GetGaindB(dev, lms.LMS_CH_TX, CH, g_cur)

        rf_mhz = (LO_HZ - NCO_FREQ_HZ) / 1e6
        print(
            f"TX @ {rf_mhz:.1f} MHz  "
            f"(host={host_sr[0]/1e6:.2f} Msps, rf={rf_sr[0]/1e6:.2f} Msps, gain={int(g_cur[0])} dB). "
            "Ctrl+C to stop."
        )

        # 5) Stream loop
        meta = lms.lms_stream_meta_t()
        meta.flushPartialPacket = False
        meta.waitForTimestamp = False
        meta.timestamp = 0

        # Keep sending until SIGINT
        # Note: LimeSuite expects number of complex samples, not I/Q integers count.
        while keep_running:
            sent = lms.LMS_SendStream(txs, buf, BUF_SAMPLES, meta, SEND_TIMEOUT_MS)
            if sent < 0:
                print(f"LMS_SendStream error: {_errstr()}", file=sys.stderr)
                break

        print("\nSIGINT detected: muting TX and shutting down safely...")

    except KeyboardInterrupt:
        print("\nSIGINT detected: muting TX and shutting down safely...")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)

    finally:
        # cleanup (graceful mute)
        try:
            if txs and getattr(txs, "handle", None):
                z = np.zeros(2 * BUF_SAMPLES, dtype=np.int16)
                meta = lms.lms_stream_meta_t()
                meta.flushPartialPacket = False
                meta.waitForTimestamp = False
                meta.timestamp = 0
                try:
                    _ = lms.LMS_SendStream(txs, z, BUF_SAMPLES, meta, SEND_TIMEOUT_MS)
                except Exception:
                    pass

                try:
                    lms.LMS_StopStream(txs)
                except Exception:
                    pass
                try:
                    lms.LMS_DestroyStream(dev, txs)
                except Exception:
                    pass
                print("TX stream stopped.")
        except Exception:
            pass

        # Disable TX channel
        try:
            if dev:
                lms.LMS_EnableChannel(dev, lms.LMS_CH_TX, CH, False)
                print("TX channel disabled.")
        except Exception:
            pass

        # Close device
        try:
            if dev:
                lms.LMS_Close(dev)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
