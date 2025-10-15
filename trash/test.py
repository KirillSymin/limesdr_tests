import numpy as np

def write_wav(samples, ssrate=128e3, dsrate=5e6, fname="symbols"):
    from scipy.signal import resample_poly
    from scipy.io.wavfile import write
    import math

    ssr = int(ssrate)
    dsr = int(dsrate)

    x = samples

    if ssr != dsr:
        g = math.gcd(ssr, dsr)
        i = dsr / g
        d = ssr / g

        x = resample_poly(x, i, d)
        print("TX signal in time domain:\r\n", x[: 5 * 39 : 39])
        print("Resampling GCD:", g)
        print("Resampling interpolation factor:", i)
        print("Resampling decimation factor:", d)
        print("Num of samples initial:", len(samples))
        print("Num of samples for SDR:", len(x))

    re_max = np.max(x.real)
    re_min = np.min(x.real)
    im_max = np.max(x.imag)
    im_min = np.min(x.imag)
    re_scale_fact = 0
    im_scale_fact = 0
    scale_fact = 0

    if re_max > abs(re_min):
        re_scale_fact = re_max
    else:
        re_scale_fact = abs(re_min)

    if im_max > abs(im_min):
        im_scale_fact = im_max
    else:
        im_scale_fact = abs(im_min)

    if re_scale_fact > im_scale_fact:
        scale_fact = re_scale_fact
    else:
        scale_fact = im_scale_fact

    amplitude = np.iinfo(np.int16).max
    amplitude = 1 << 11
    print("DAC amplitume:", amplitude)
    scale_fact = (amplitude - 8) / scale_fact  # avoid saturation
    print("unscaled:", x[:5])
    rex = x.real * scale_fact
    imx = x.imag * scale_fact
    rex = rex.astype(np.int16)
    imx = imx.astype(np.int16)
    print("scaled:", rex[:5], imx[:5])
    wav = np.array([rex, imx]).T

    write(fname + ".wav", dsr, wav)
    return x


FRQ_SIN = 2048
LEN_SIN = 1 << 16

sr = 128e3

t = np.arange(LEN_SIN) / sr
x = np.sin(FRQ_SIN * t) + 1j * np.cos(FRQ_SIN * t)

print(x)

write_wav(x, sr, WAV_SMP)
