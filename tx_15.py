#!/usr/bin/env python3
import signal
import sys
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX  # direction constants

# -------------------- Constants (match your C example) --------------------
CH                 = 0              # TX channel A
HOST_SR_HZ         = 5_000_000      # 5 Msps host-side
OVERSAMPLE         = 8              # target RF ≈ HOST_SR * 8 = 40 Msps (Lime internal)
TX_LPF_BW_HZ       = 50_000_000     # 50 MHz
LO_HZ              = 30_000_000     # 30 MHz LO
NCO_FREQ_HZ        = 15_000_000     # 15 MHz NCO magnitude
NCO_DOWNCONVERT    = True           # RF = LO - NCO  (so set BB = -NCO)
TX_GAIN_DB         = 40             # moderate TX gain
BUF_SAMPLES        = 8192           # chunk size (complex samples)
SEND_TIMEOUT_US    = 1_000_000      # 1 s
TONE_SCALE         = 0.70           # 70% full scale
STREAM_BUFFER_LEN  = 131072         # driver hint (samples); adjust if needed

keep_running = True
def on_sigint(signum, frame):
    global keep_running
    keep_running = False
signal.signal(signal.SIGINT, on_sigint)

# -------------------- Open first Lime device via Soapy --------------------
# If you have multiple SDRs, pass args like dict(driver="lime", serial="XXXX")
devs = SoapySDR.Device.enumerate(dict(driver="lime"))
if not devs:
    print("No LimeSDR found", file=sys.stderr)
    sys.exit(1)

sdr = SoapySDR.Device(devs[0])

# -------------------- Basic TX setup --------------------
# Enable TX channel (Soapy enables on stream activate, but we set params first)
sdr.setSampleRate(SOAPY_SDR_TX, CH, HOST_SR_HZ)
# (Lime oversampling is chosen internally based on requested host SR; aiming for ~40 Msps NCO range)

sdr.setBandwidth(SOAPY_SDR_TX, CH, TX_LPF_BW_HZ)
sdr.setGain(SOAPY_SDR_TX, CH, TX_GAIN_DB)

# RF LO at 30 MHz
sdr.setFrequency(SOAPY_SDR_TX, CH, "RF", LO_HZ)

# NCO (BB) at -15 MHz for down-convert so RF = LO - NCO = 15 MHz
bb = -NCO_FREQ_HZ if NCO_DOWNCONVERT else +NCO_FREQ_HZ
sdr.setFrequency(SOAPY_SDR_TX, CH, "BB", bb)

# Optional: printbacks (best-effort; not all drivers provide readback)
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
# (Many drivers don’t expose an NCO index/readback; we skip it.)

# -------------------- TX stream --------------------
# Format 'CS16' = 16-bit IQ (matches LMS_FMT_I16)
st_args = {"bufferLength": STREAM_BUFFER_LEN}  # driver-specific hint; safe to omit
tx_stream = sdr.setupStream(SOAPY_SDR_TX, "CS16", [CH], st_args)
sdr.activateStream(tx_stream)  # start

print(f"TX stream started (bufferLength={STREAM_BUFFER_LEN}, fmt=CS16).")

# Build a constant IQ buffer: I=0.7 FS, Q=0 -> pure tone at NCO after up/downconversion
I = np.int16(TONE_SCALE * 32767)
Q = np.int16(0)
iq = np.empty(BUF_SAMPLES, dtype=np.complex64)  # build in float then view as int16 interleaved
iq.real = I
iq.imag = Q
# Convert to interleaved int16 (Soapy expects raw bytes)
iq_i16 = np.empty(2*BUF_SAMPLES, dtype=np.int16)
iq_i16[0::2] = I
iq_i16[1::2] = Q

print(f"TX @ {(LO_HZ - (NCO_FREQ_HZ if NCO_DOWNCONVERT else -NCO_FREQ_HZ))/1e6:.1f} MHz "
      f"(host={HOST_SR_HZ/1e6:.2f} Msps, gain={TX_GAIN_DB} dB). Ctrl+C to stop.")

# -------------------- Stream loop --------------------
while keep_running:
    # writeStream expects a pointer-like object; numpy gives a buffer interface
    sr = sdr.writeStream(tx_stream, [iq_i16], BUF_SAMPLES, timeoutUs=SEND_TIMEOUT_US)
    if sr.ret < 0:
        print(f"writeStream error: {sr.ret}", file=sys.stderr)
        break

print("\nSIGINT detected: muting TX and shutting down safely...")

# -------------------- Cleanup (graceful mute) --------------------
try:
    # push one zero buffer so last burst is silence
    z = np.zeros(2*BUF_SAMPLES, dtype=np.int16)
    _ = sdr.writeStream(tx_stream, [z], BUF_SAMPLES, timeoutUs=SEND_TIMEOUT_US)
except Exception:
    pass

try:
    sdr.deactivateStream(tx_stream)
    sdr.closeStream(tx_stream)
    print("TX stream stopped.")
except Exception:
    pass
