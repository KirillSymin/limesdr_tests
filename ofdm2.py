#!/usr/bin/env python3
"""
OFDM TX for LimeSDR (SoapySDR) with CLI-settable parameters.

- Supports BPSK, QPSK/8PSK/32PSK, 16QAM/64QAM
- Optional pilots (BPSK on a comb), adjustable CP, FFT size, occupied BW, etc.
- Can derive fs from target symbol duration (no-CP) or accept explicit sample rate
- Prints derived subcarrier spacing, symbol timing, and data rates

NOTE: This is a barebones transmitter: no preamble/equalization/training.
"""

import argparse
import sys
import signal
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX  # direction constants

# -------------------- CLI --------------------
def parse_args():
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Minimal OFDM TX for LimeSDR with command-line parameters."
    )

    # Radio / device
    p.add_argument("--driver", default="lime", help="SoapySDR driver")
    p.add_argument("--channel", type=int, default=0, help="TX channel index")
    p.add_argument("--sample-rate", type=float, default=None,
                   help="Host complex sample rate in sps (overrides symbol-duration if given)")
    p.add_argument("--tx-lpf-bw", type=float, default=20e6, help="TX LPF bandwidth (Hz)")
    p.add_argument("--lo-hz", type=float, default=30e6, help="RF LO frequency (Hz)")
    p.add_argument("--nco-hz", type=float, default=15e6, help="BB NCO frequency (Hz)")
    p.add_argument("--nco-downconvert", type=int, default=1, choices=[0,1],
                   help="If 1: RF=LO-NCO (BB=-NCO), else RF=LO+NCO (BB=+NCO)")
    p.add_argument("--tx-gain", type=float, default=40.0, help="TX gain (dB)")
    p.add_argument("--send-timeout-us", type=int, default=1_000_000, help="writeStream timeout (us)")
    p.add_argument("--tx-scale", type=float, default=0.30, help="TX amplitude scale (watch PAPR)")
    p.add_argument("--chunk-syms", type=int, default=16, help="OFDM symbols per writeStream")

    # OFDM core
    p.add_argument("--fft-len", type=int, default=64, help="IFFT size (N)")
    p.add_argument("--cp-frac", type=float, default=0.25, help="Cyclic prefix length as a fraction of N")
    p.add_argument("--edge-guard", type=int, default=0, help="Guard bins left unused at the edges (each side)")
    p.add_argument("--num-pilots", type=int, default=6, help="Number of pilot subcarriers (total)")
    p.add_argument("--num-payload", type=int, default=22, help="Number of payload subcarriers (total)")
    p.add_argument("--used-tones", type=int, default=None,
                   help="Total occupied subcarriers (overrides num-pilots+num-payload if given)")
    p.add_argument("--occupied-bw-khz", type=float, default=56.0,
                   help="Target occupied bandwidth (kHz). Used only if used-tones is not explicitly set and num-* not set.")
    p.add_argument("--symbol-duration-ms", type=float, default=0.625,
                   help="OFDM symbol duration *without CP* (ms). Used to derive fs if --sample-rate is not given.")
    p.add_argument("--freq-offset-khz", type=float, default=2.0,
                   help="Optional baseband frequency offset to apply (kHz, positive rotates +f)")

    # Modulation
    p.add_argument("--mod", default="16QAM",
                   choices=["BPSK","QPSK","8PSK","32PSK","16QAM","64QAM"],
                   help="Constellation for payload subcarriers")

    # Misc
    p.add_argument("--rng-seed", type=int, default=12345, help="Random seed")
    p.add_argument("--softclip-alpha", type=float, default=1.5,
                   help="tanh soft clip strength; 0 disables")

    return p.parse_args()

# -------------------- Graceful shutdown --------------------
keep_running = True
def on_sigint(signum, frame):
    global keep_running
    keep_running = False
signal.signal(signal.SIGINT, on_sigint)

# -------------------- Modulations --------------------
def bits_per_symbol(mod_name: str) -> int:
    m = mod_name.upper()
    if m == "BPSK":  return 1
    if m == "QPSK":  return 2
    if m == "8PSK":  return 3
    if m == "32PSK": return 5
    if m == "16QAM": return 4
    if m == "64QAM": return 6
    raise ValueError(f"Unsupported modulation {mod_name}")

def gen_symbols(mod_name: str, n_syms: int, rng: np.random.Generator) -> np.ndarray:
    m = mod_name.upper()
    if m == "BPSK":
        bits = rng.integers(0, 2, size=n_syms, dtype=np.int8)
        sym = 2*bits - 1  # {-1,+1}
        return sym.astype(np.complex64)
    if m in ("QPSK","8PSK","32PSK"):
        M = {"QPSK":4, "8PSK":8, "32PSK":32}[m]
        # Random integer symbols with unit-magnitude PSK
        idx = rng.integers(0, M, size=n_syms, dtype=np.int32)
        phase = 2*np.pi*idx / M
        return np.exp(1j*phase).astype(np.complex64)
    if m in ("16QAM","64QAM"):
        M = {"16QAM":16, "64QAM":64}[m]
        k = int(np.sqrt(M))
        if k*k != M:
            raise ValueError("Only square QAM supported")
        # Gray-like square mapping on levels {-k+1, ..., -1, +1, ..., +k-1}
        levels = np.arange(-(k-1), (k-1)+1, 2, dtype=np.float32)
        # Choose I/Q indices
        idx = rng.integers(0, M, size=n_syms, dtype=np.int32)
        i_idx = idx % k
        q_idx = idx // k
        i = levels[i_idx]
        q = levels[q_idx]
        # Normalization for unit average power
        if M == 16:
            scale = np.sqrt(10.0).astype(np.float32)
        else:  # 64
            scale = np.sqrt(42.0).astype(np.float32)
        const = (i + 1j*q) / scale
        return const.astype(np.complex64)
    raise ValueError(f"Unsupported modulation {mod_name}")

# -------------------- Helpers --------------------
def soft_clip(x, alpha=2.0):
    if alpha <= 0:
        return x
    peak = np.max(np.abs(x)) + 1e-9
    y = np.tanh(alpha * (x / peak)) / np.tanh(alpha)
    # Energy re-normalization
    rms_x = np.sqrt(np.mean(np.abs(x)**2)) + 1e-12
    rms_y = np.sqrt(np.mean(np.abs(y)**2)) + 1e-12
    y *= (rms_x / rms_y)
    return y

def float_to_cs16(iq_f):
    i16 = np.empty(2*iq_f.size, dtype=np.int16)
    i = np.clip(np.real(iq_f) * 32767.0, -32768, 32767).astype(np.int16)
    q = np.clip(np.imag(iq_f) * 32767.0, -32768, 32767).astype(np.int16)
    i16[0::2] = i
    i16[1::2] = q
    return i16

def build_subcarrier_plan(fft_n, used_tones, edge_guard, n_pilots):
    assert used_tones % 2 == 0, "USED_TONES must be even (split across +/-)."
    max_used = fft_n - 2 - 2*edge_guard
    assert used_tones <= max_used, f"Too many used tones for FFT={fft_n}, guard={edge_guard}."

    half_used = used_tones // 2
    pos_bins = np.arange(1 + edge_guard, 1 + edge_guard + half_used, dtype=np.int32)
    neg_bins = fft_n - np.arange(1 + edge_guard, 1 + edge_guard + half_used, dtype=np.int32)
    used_bins = np.concatenate([pos_bins, neg_bins])  # order doesn't matter for randomness

    n_pilots = int(np.clip(n_pilots, 0, used_tones))
    # choose pilot bins approximately evenly across 'used_bins'
    if n_pilots > 0:
        pilot_sel = np.unique(np.round(np.linspace(0, used_bins.size - 1, n_pilots, dtype=np.int32)))
        pilot_bins = used_bins[pilot_sel]
    else:
        pilot_bins = np.array([], dtype=np.int32)

    data_bins = np.setdiff1d(used_bins, pilot_bins, assume_unique=False)
    return data_bins, pilot_bins

def make_ofdm_symbol(data_syms, pilot_sym, data_bins, pilot_bins, fft_n, cp_len):
    X = np.zeros(fft_n, dtype=np.complex64)
    # map data
    X[data_bins] = data_syms
    # map pilots
    if pilot_bins.size:
        X[pilot_bins] = pilot_sym
    # IFFT and CP
    x = np.fft.ifft(X, n=fft_n).astype(np.complex64)
    return np.concatenate((x[-cp_len:], x))

# -------------------- Main --------------------
def main():
    args = parse_args()
    rng = np.random.default_rng(args.rng_seed)

    FFT_N = args.fft_len
    CP_LEN = int(round(args.cp_frac * FFT_N))
    CP_FRAC = CP_LEN / float(FFT_N)

    # Derive sample rate and spacing
    if args.sample_rate is not None:
        fs = float(args.sample_rate)
        delta_f = fs / FFT_N
        T_noCP = 1.0 / delta_f
    else:
        T_noCP = float(args.symbol_duration_ms) * 1e-3
        delta_f = 1.0 / T_noCP
        fs = delta_f * FFT_N

    Tsym = T_noCP * (1.0 + CP_FRAC)

    # Decide USED_TONES
    if args.used_tones is not None:
        USED_TONES = int(args.used_tones)
        n_payload = int(args.num_payload) if args.num_payload is not None else USED_TONES - int(args.num_pilots)
        n_pilots = int(args.num_pilots)
        if n_payload + n_pilots != USED_TONES:
            # reconcile: clamp payload to fill remaining
            n_payload = max(0, USED_TONES - n_pilots)
    elif args.num_payload is not None and args.num_pilots is not None:
        n_payload = int(args.num_payload)
        n_pilots = int(args.num_pilots)
        USED_TONES = n_payload + n_pilots
    else:
        # Infer from occupied BW
        used_from_bw = int(round((args.occupied_bw_khz * 1e3) / delta_f))
        # Make even and at least 2
        USED_TONES = max(2, used_from_bw - (used_from_bw % 2))
        n_pilots = int(min(6, USED_TONES//4))
        n_payload = USED_TONES - n_pilots

    # Build mapping
    data_bins, pilot_bins = build_subcarrier_plan(
        fft_n=FFT_N,
        used_tones=USED_TONES,
        edge_guard=args.edge_guard,
        n_pilots=n_pilots
    )
    assert data_bins.size == n_payload, \
        f"Data bins {data_bins.size} != requested payload {n_payload}. (Pilots={pilot_bins.size}, used={USED_TONES})"

    # Rates
    bps_mod = bits_per_symbol(args.mod)
    raw_bps_noCP = n_payload * bps_mod * delta_f                        # like your table (ignores CP)
    net_bps_withCP = (n_payload * bps_mod) / Tsym                       # accounts for CP overhead

    # Frequency offset rotator
    f_off = float(args.freq_offset_khz) * 1e3
    rotator = np.exp(1j * 2*np.pi * f_off * np.arange(FFT_N + CP_LEN) / fs).astype(np.complex64) \
              if abs(f_off) > 0 else None

    # -------- Open device --------
    devs = SoapySDR.Device.enumerate(dict(driver=args.driver))
    if not devs:
        print("No SDR found for driver:", args.driver, file=sys.stderr)
        sys.exit(1)
    sdr = SoapySDR.Device(devs[0])

    CH = args.channel
    sdr.setSampleRate(SOAPY_SDR_TX, CH, fs)
    sdr.setBandwidth(SOAPY_SDR_TX, CH, args.tx_lpf_bw)
    sdr.setGain(SOAPY_SDR_TX, CH, args.tx_gain)
    sdr.setFrequency(SOAPY_SDR_TX, CH, "RF", args.lo_hz)
    bb = -args.nco_hz if args.nco_downconvert else +args.nco_hz
    sdr.setFrequency(SOAPY_SDR_TX, CH, "BB", bb)

    # Log settings
    try:
        host_sr = sdr.getSampleRate(SOAPY_SDR_TX, CH)
        print(f"Set/Get: SampleRate host={host_sr/1e6:.6f} Msps")
    except Exception:
        pass
    try:
        print(f"Set/Get: TX Gain = {sdr.getGain(SOAPY_SDR_TX, CH):.0f} dB")
    except Exception:
        pass
    try:
        print(f"Set/Get: RF LO = {sdr.getFrequency(SOAPY_SDR_TX, CH, 'RF')/1e6:.6f} MHz")
    except Exception:
        pass

    tx_stream = sdr.setupStream(SOAPY_SDR_TX, "CS16", [CH])
    sdr.activateStream(tx_stream)
    print("TX stream started (CS16).")

    print(f"OFDM config: N={FFT_N}, CP={CP_LEN} ({CP_FRAC:.3f} of N), Δf={delta_f/1e3:.3f} kHz, "
          f"Tsym(noCP)={T_noCP*1e3:.3f} ms, Tsym={Tsym*1e3:.3f} ms")
    print(f"Mapping: used={USED_TONES} (payload={n_payload}, pilots={n_pilots}), edgeGuard={args.edge_guard}, DC unused")
    print(f"Rates: raw(no-CP)={raw_bps_noCP/1e3:.3f} kb/s, net(with-CP)={net_bps_withCP/1e3:.3f} kb/s, mod={args.mod}")
    rf_center = args.lo_hz - (args.nco_hz if args.nco_downconvert else -args.nco_hz)
    print(f"Transmitting at ~{rf_center/1e6:.6f} MHz (fs={fs/1e6:.6f} Msps). Ctrl+C to stop.")

    # -------- Streaming loop --------
    CHUNK_SYMS = args.chunk_syms
    SYMBOL_LEN = FFT_N + CP_LEN

    try:
        while keep_running:
            # generate random payload constellation for all data bins across CHUNK_SYMS
            n_data_syms = CHUNK_SYMS * data_bins.size
            payload = gen_symbols(args.mod, n_data_syms, rng).reshape(CHUNK_SYMS, data_bins.size)

            # simple pilot pattern: BPSK +1/-1 toggling each OFDM symbol
            pilot_vals = (1.0 if n_pilots == 0 else 1.0).astype(np.complex64) if n_pilots else 0.0

            syms_td = np.empty((CHUNK_SYMS, SYMBOL_LEN), dtype=np.complex64)
            for n in range(CHUNK_SYMS):
                pilot_sym = (1.0 if (n % 2 == 0) else -1.0) if n_pilots else 0.0
                sym_td = make_ofdm_symbol(
                    data_syms=payload[n],
                    pilot_sym=np.complex64(pilot_sym),
                    data_bins=data_bins,
                    pilot_bins=pilot_bins,
                    fft_n=FFT_N,
                    cp_len=CP_LEN
                )
                if rotator is not None:
                    sym_td = (sym_td * rotator).astype(np.complex64)
                syms_td[n, :] = sym_td

            buf = syms_td.reshape(-1)
            # normalize RMS then scale
            rms = np.sqrt(np.mean(np.abs(buf)**2)) + 1e-12
            buf = (args.tx_scale / rms) * buf

            # optional soft clipping
            buf = soft_clip(buf, alpha=args.softclip_alpha).astype(np.complex64)

            iq_i16 = float_to_cs16(buf)
            num_samps = buf.size
            sr = sdr.writeStream(tx_stream, [iq_i16], num_samps, timeoutUs=args.send_timeout_us)
            if sr.ret < 0:
                print(f"writeStream error: {sr.ret}", file=sys.stderr)
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("\nSIGINT detected: muting TX and shutting down safely...")
        try:
            z = np.zeros(2*SYMBOL_LEN, dtype=np.int16)
            _ = sdr.writeStream(tx_stream, [z], SYMBOL_LEN, timeoutUs=args.send_timeout_us)
        except Exception:
            pass
        try:
            sdr.deactivateStream(tx_stream)
            sdr.closeStream(tx_stream)
            print("TX stream stopped.")
        except Exception:
            pass

if __name__ == "__main__":
    main()
