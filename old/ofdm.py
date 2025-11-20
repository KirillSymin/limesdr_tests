#!/usr/bin/env python3
"""
Minimal OFDM transmitter for LimeSDR via SoapySDR.
- Generates continuous 16-QAM OFDM symbols with cyclic prefix
- Centers at baseband (BB=±NCO as you already set), same RF chain as your tone example
- No pilots/equalization/training: this is a barebones TX demo

Tested with numpy + SoapySDR Python bindings.

Tip: If you want pilots/preamble later, insert known BPSK symbols on a comb of subcarriers
and/or prepend LTS/STS symbols to each chunk.
"""

import signal
import sys
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX  # direction constants

# -------------------- Radio constants (copied/adapted from your script) --------------------
CH                 = 0
HOST_SR_HZ         = 5_000_000      # host-side complex sample rate
TX_LPF_BW_HZ       = 20_000_000
LO_HZ              = 30_000_000
NCO_FREQ_HZ        = 15_000_000
NCO_DOWNCONVERT    = True           # RF = LO - NCO (so BB set to -NCO)
TX_GAIN_DB         = 40
SEND_TIMEOUT_US    = 1_000_000
TX_SCALE           = 0.30           # overall amplitude (OFDM has high PAPR! keep small)
CHUNK_SYMS         = 16             # how many OFDM symbols per writeStream call

# -------------------- OFDM parameters --------------------
FFT_N              = 512            # IFFT size
CP_LEN             = 128            # cyclic prefix length (samples)
USED_TONES         = 300            # number of occupied subcarriers (even, excludes DC)
EDGE_GUARD         = 0              # set >0 to leave guard bands at edges (not required here)
MOD_BITS           = 4              # 16-QAM (4 bits per subcarrier)
RNG_SEED           = 12345

# Derivations/checks
assert USED_TONES % 2 == 0, "USED_TONES must be even (split across +/-)."
assert USED_TONES <= FFT_N - 2 - 2*EDGE_GUARD, "Too many used tones for chosen FFT/guards."
SYMBOL_LEN = FFT_N + CP_LEN

# -------------------- Graceful shutdown --------------------
keep_running = True
def on_sigint(signum, frame):
    global keep_running
    keep_running = False
signal.signal(signal.SIGINT, on_sigint)

# -------------------- Helpers: 16-QAM (Gray-ish) mapping --------------------
# Map two bits -> amplitude in {-3,-1,+1,+3} using Gray-like mapping
# 00 -> -3, 01 -> -1, 11 -> +1, 10 -> +3
_lut2 = np.array([-3, -1, +1, +3], dtype=np.float32)
def _amps_from_2bits(b2):
    return _lut2[b2]

def qam16_symbols(n_syms, rng):
    """Return n_syms 16-QAM unit-average-power complex symbols."""
    bits = rng.integers(0, 2, size=(n_syms, 4), dtype=np.int8)
    i_idx = (bits[:,0] << 1) | bits[:,1]
    q_idx = (bits[:,2] << 1) | bits[:,3]
    i = _amps_from_2bits(i_idx)
    q = _amps_from_2bits(q_idx)
    const = (i + 1j*q) / np.sqrt(10.0, dtype=np.float32)  # normalize E{|s|^2}=1
    return const.astype(np.complex64)

# -------------------- Helpers: OFDM mapping --------------------
# Build static index maps for placing QAM onto positive/negative subcarriers
half_used = USED_TONES // 2
pos_bins = np.arange(1 + EDGE_GUARD, 1 + EDGE_GUARD + half_used, dtype=np.int32)          # [1..half_used]
neg_bins = FFT_N - np.arange(1 + EDGE_GUARD, 1 + EDGE_GUARD + half_used, dtype=np.int32)  # [-1..-half_used]

def map_to_subcarriers(data_syms):
    """
    data_syms: complex array of length USED_TONES
    Returns: frequency-domain vector X[k], k=0..FFT_N-1 with DC and guards zeroed.
    """
    X = np.zeros(FFT_N, dtype=np.complex64)
    X[pos_bins] = data_syms[:half_used]
    X[neg_bins] = data_syms[half_used:]
    # DC (k=0) and any unused carriers remain zero
    return X

def ofdm_symbol_from_data(data_syms):
    """
    Build time-domain OFDM symbol with cyclic prefix from USED_TONES QAM symbols.
    """
    X = map_to_subcarriers(data_syms)
    x = np.fft.ifft(X, n=FFT_N).astype(np.complex64)  # numpy ifft includes 1/N
    # Add CP (copy last CP_LEN samples to front)
    sym = np.concatenate((x[-CP_LEN:], x))
    return sym

def soft_clip(x, alpha=2.0):
    """
    Smooth limiter: scales toward tanh to reduce PAPR gently.
    Set alpha=0 to disable. Keep small to avoid distortion.
    """
    if alpha <= 0:
        return x
    # Normalize to ~unit peak before tanh, then rescale to preserve RMS roughly
    peak = np.max(np.abs(x)) + 1e-9
    y = np.tanh(alpha * (x / peak)) / np.tanh(alpha)
    # Re-normalize RMS to original
    rms_x = np.sqrt(np.mean(np.abs(x)**2)) + 1e-12
    rms_y = np.sqrt(np.mean(np.abs(y)**2)) + 1e-12
    y *= (rms_x / rms_y)
    return y

def float_to_cs16(iq_f):
    """Convert complex float32 [-1..+1] approx to interleaved int16 IQ."""
    i16 = np.empty(2*iq_f.size, dtype=np.int16)
    i = np.clip(np.real(iq_f) * 32767.0, -32768, 32767).astype(np.int16)
    q = np.clip(np.imag(iq_f) * 32767.0, -32768, 32767).astype(np.int16)
    i16[0::2] = i
    i16[1::2] = q
    return i16

# -------------------- Open first Lime device --------------------
devs = SoapySDR.Device.enumerate(dict(driver="lime"))
if not devs:
    print("No LimeSDR found", file=sys.stderr)
    sys.exit(1)
sdr = SoapySDR.Device(devs[0])

# -------------------- TX setup --------------------
sdr.setSampleRate(SOAPY_SDR_TX, CH, HOST_SR_HZ)
sdr.setBandwidth(SOAPY_SDR_TX, CH, TX_LPF_BW_HZ)
sdr.setGain(SOAPY_SDR_TX, CH, TX_GAIN_DB)
sdr.setFrequency(SOAPY_SDR_TX, CH, "RF", LO_HZ)
bb = -NCO_FREQ_HZ if NCO_DOWNCONVERT else +NCO_FREQ_HZ
sdr.setFrequency(SOAPY_SDR_TX, CH, "BB", bb)

try:
    host_sr = sdr.getSampleRate(SOAPY_SDR_TX, CH)
    print(f"Set/Get: SampleRate host={host_sr/1e6:.2f} Msps")
except Exception:
    pass
try:
    print(f"Set/Get: TX Gain = {sdr.getGain(SOAPY_SDR_TX, CH):.0f} dB")
except Exception:
    pass
try:
    print(f"Set/Get: LO = {sdr.getFrequency(SOAPY_SDR_TX, CH, 'RF')/1e6:.6f} MHz")
except Exception:
    pass

tx_stream = sdr.setupStream(SOAPY_SDR_TX, "CS16", [CH])
sdr.activateStream(tx_stream)
print("TX stream started (fmt=CS16).")

print(f"OFDM params: N={FFT_N}, CP={CP_LEN}, used={USED_TONES}, Δf={HOST_SR_HZ/FFT_N/1e3:.2f} kHz, "
      f"symbolLen={SYMBOL_LEN} samples, chunk={CHUNK_SYMS} symbols.")

center_rf = LO_HZ - (NCO_FREQ_HZ if NCO_DOWNCONVERT else -NCO_FREQ_HZ)
print(f"Transmitting OFDM @ {center_rf/1e6:.3f} MHz (host={HOST_SR_HZ/1e6:.2f} Msps). Ctrl+C to stop.")

# -------------------- Streaming loop --------------------
rng = np.random.default_rng(RNG_SEED)

try:
    while keep_running:
        # Generate CHUNK_SYMS symbols worth of data
        # 1) Random QAM data for all used tones across all symbols
        data = qam16_symbols(CHUNK_SYMS * USED_TONES, rng).reshape(CHUNK_SYMS, USED_TONES)

        # 2) Build time-domain symbols w/ CP and stack
        syms_td = np.empty((CHUNK_SYMS, SYMBOL_LEN), dtype=np.complex64)
        for n in range(CHUNK_SYMS):
            sym = ofdm_symbol_from_data(data[n])
            syms_td[n, :] = sym

        # 3) Normalize average RMS to TX_SCALE (protect against PAPR)
        buf = syms_td.reshape(-1)
        rms = np.sqrt(np.mean(np.abs(buf)**2)) + 1e-12
        buf = (TX_SCALE / rms) * buf

        # 4) Optional gentle clipping to reduce peaks (comment out to disable)
        buf = soft_clip(buf, alpha=1.5)

        # 5) Convert to interleaved CS16 and send
        iq_i16 = float_to_cs16(buf.astype(np.complex64))
        num_samps = buf.size
        sr = sdr.writeStream(tx_stream, [iq_i16], num_samps, timeoutUs=SEND_TIMEOUT_US)
        if sr.ret < 0:
            print(f"writeStream error: {sr.ret}", file=sys.stderr)
            break

except KeyboardInterrupt:
    pass
finally:
    print("\nSIGINT detected: muting TX and shutting down safely...")
    try:
        # one chunk of zeros to mute
        z = np.zeros(2*SYMBOL_LEN, dtype=np.int16)
        _ = sdr.writeStream(tx_stream, [z], SYMBOL_LEN, timeoutUs=SEND_TIMEOUT_US)
    except Exception:
        pass
    try:
        sdr.deactivateStream(tx_stream)
        sdr.closeStream(tx_stream)
        print("TX stream stopped.")
    except Exception:
        pass
