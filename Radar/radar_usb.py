# -*- coding: utf-8 -*-
"""
Created on Sat May  2 17:29:25 2026

@author: arnesons
"""

#!/usr/bin/env python3
"""
pluto_true_radar_fmcw_usb_ascii_bgsub_pi3.py

True reflected-target FMCW radar for Pluto Plus, running on Raspberry Pi 3.

Changes from UDP version:
- Outputs over USB serial instead of UDP
- Keeps the same ASCII packet format:
      range_m,angle_deg,quality\n
- Optimized for Raspberry Pi 3 CPU/RAM limits
- Designed for a 16 GB Raspberry Pi microSD card setup

Hardware assumption:
- Raspberry Pi 3 talks to Pluto Plus using libiio over USB/Ethernet gadget
- Pluto URI is usually ip:192.168.2.1
- Raspberry Pi sends serial data to STM32 over USB serial
- STM32 appears on Pi as /dev/ttyACM0 or /dev/ttyUSB0

Packet example sent to STM32:
    2.431,-7.52,0.81\n
"""

import time
import numpy as np
import adi
import serial
import os
import glob


# -------------------------------------------------
# Constants
# -------------------------------------------------
C = 299_792_458.0


# -------------------------------------------------
# User settings
# -------------------------------------------------

# Pluto Plus URI.
# This should work if the Pi can ping 192.168.2.1.
URI = "ip:192.168.2.1"

# USB serial output to STM32.
# Most STM32 virtual COM ports show up as /dev/ttyACM1.
# USB-to-UART adapters usually show up as /dev/ttyUSB0.
USB_PORT = None
USB_BAUD = 9600

# RF settings.
# These are lowered compared to the original 10 MSPS version
# so the Raspberry Pi 3 has a better chance of keeping up.
FC = 2.4e9
FS = 2_000_000
RF_BW = 2_000_000

# FMCW chirp settings.
# Lower sweep bandwidth reduces processing load, but also reduces range resolution.
T_CHIRP = 400e-6
B_SWEEP = 1e6
TX_AMP_CHIRP = 0.22
TX_GAIN = -35

# RX gains.
RX_GAIN0 = 0
RX_GAIN1 = 0

# Physical RX spacing in meters.
# 0.030 m is about lambda/4 at 2.4 GHz.
# Use the real measured spacing between RX phase centers.
D_RX = 0.030

# Frame formatting.
PRE_F0 = 200e3
PRE_N = 2048
PRE_AMP = 0.50
GUARD0_N = 256
CHIRP_EDGE_GUARD = 64
TX_FRAME_REPEATS = 64

# Acquisition settings optimized for Raspberry Pi 3.
RX_BUF = 131072
DISCARD_RX_READS = 2
ACCUM_READS = 3
CHIRPS_TO_AVG = 12

# Sync.
PEAK_THRESH_RATIO = 0.80

# True radar target search band.
FB_MIN_HZ = 30.0
FB_MAX_HZ = 3000.0

# Quality / target detection thresholds.
MIN_PEAK_TO_MEDIAN = 0.0
MIN_COHERENCE = 0.0

# Background subtraction.
BG_ALPHA = 0.0
BG_INIT_FRAMES = 0
BG_FREEZE_QUALITY = 0.999999

# Send pacing.
# 0.10 s = 10 packets per second max.
# This is easier for the STM32 and Pi than 50 packets per second.
MIN_SEND_PERIOD_S = 0.05

# Debug print to Pi terminal.
# Leave True while testing. Set False after it works.
PRINT_PACKETS = True


# -------------------------------------------------
# Signal generation helpers
# -------------------------------------------------

def make_chirp(fs, T, B, amp=1.0):
    N = int(round(fs * T))
    t = np.arange(N, dtype=np.float64) / fs
    slope = B / T
    phase = 2.0 * np.pi * ((-B / 2.0) * t + 0.5 * slope * t * t)
    return (amp * np.exp(1j * phase)).astype(np.complex64)


def make_preamble_tone(fs, f0=200e3, N=2048, amp=0.5):
    n = np.arange(N, dtype=np.float64)
    pre = amp * np.exp(1j * 2.0 * np.pi * (f0 / fs) * n)
    return pre.astype(np.complex64)


def two_way_range_from_fb(fb_hz, B, T):
    """
    True reflected-target FMCW radar range.

        fb = S * tau
        tau = 2R / c
        R = c * fb / (2S)
    """
    slope = B / T
    return (C * fb_hz) / (2.0 * slope)


# -------------------------------------------------
# FFT / sync helpers
# -------------------------------------------------

def next_pow2(n):
    p = 1
    while p < n:
        p <<= 1
    return p


def fft_correlate(sig, ref):
    sig = sig.astype(np.complex128, copy=False)
    ref = ref.astype(np.complex128, copy=False)

    nfft = next_pow2(sig.size + ref.size - 1)

    sig_fft = np.fft.fft(sig, nfft)
    ref_fft = np.fft.fft(np.conjugate(ref[::-1]), nfft)

    return np.fft.ifft(sig_fft * ref_fft)


def earliest_strong_peak_start_frame_aligned(
    sig,
    ref,
    nframe,
    thresh_ratio=0.2,
    min_start=0,
    max_candidates=1000
):
    corr = fft_correlate(sig, ref)
    mag = np.abs(corr)

    maxv = float(np.max(mag))
    if maxv <= 0:
        return None

    thresh = thresh_ratio * maxv
    ref_len = ref.size

    starts = np.arange(mag.size) - (ref_len - 1)

    idx = np.where((starts >= min_start) & (mag >= thresh))[0]

    if idx.size == 0:
        k = int(np.argmax(mag))
        start = int(max(0, min(starts[k], sig.size - 1)))
        return start

    idx = idx[np.argsort(starts[idx])]
    idx = idx[:max_candidates]

    cand_starts = starts[idx].astype(int)
    cand_mags = mag[idx]

    frame_mod = cand_starts % nframe
    mod_err = np.minimum(frame_mod, nframe - frame_mod)

    score = (mod_err * 1e6) + cand_starts - (cand_mags * 1e-2)

    best_i = int(np.argmin(score))
    best_start = int(max(0, min(cand_starts[best_i], sig.size - 1)))

    return best_start


def coherence_metric(x):
    x = x.astype(np.complex128, copy=False)
    x = x - np.mean(x)

    if x.size < 2:
        return 0.0

    p = np.conjugate(x[:-1]) * x[1:]

    return float(np.abs(np.mean(p)) / (np.mean(np.abs(p)) + 1e-12))


def beat_spectrum(beat, fs, fmin_hz, fmax_hz):
    """
    Build positive-frequency beat spectrum in the desired band.

    Returns:
        frequency axis,
        complex spectrum,
        magnitude spectrum
    """
    x = beat.astype(np.complex128, copy=False)
    x = x - np.mean(x)

    window = np.hanning(x.size)
    xw = x * window

    nfft = next_pow2(x.size * 4)

    X = np.fft.fft(xw, nfft)
    mag = np.abs(X)
    freqs = np.fft.fftfreq(nfft, d=1.0 / fs)

    pos = freqs >= 0
    freqs = freqs[pos]
    X = X[pos]
    mag = mag[pos]

    band = (freqs >= fmin_hz) & (freqs <= fmax_hz)

    return freqs[band], X[band], mag[band]


# -------------------------------------------------
# Background subtraction helpers
# -------------------------------------------------

def update_background(bg, mag, alpha):
    if bg is None:
        return mag.copy()

    return alpha * bg + (1.0 - alpha) * mag


def subtract_background(mag, bg):
    if bg is None:
        return mag.copy()

    out = mag - bg
    out[out < 0.0] = 0.0

    return out


def detect_target_bin_with_bg(beat0, beat1, fs, fmin_hz, fmax_hz, bg0, bg1):
    """
    Detect target beat bin using background-subtracted combined RX magnitude.
    """
    freqs0, X0, mag0 = beat_spectrum(beat0, fs, fmin_hz, fmax_hz)
    freqs1, X1, mag1 = beat_spectrum(beat1, fs, fmin_hz, fmax_hz)

    if freqs0.size < 10:
        return None

    if freqs1.size != freqs0.size:
        return None

    mag0_sub = subtract_background(mag0, bg0)
    mag1_sub = subtract_background(mag1, bg1)

    mag_sum_sub = mag0_sub + mag1_sub

    k = int(np.argmax(mag_sum_sub))
    fb = float(freqs0[k])

    peak = float(mag_sum_sub[k] + 1e-12)
    med = float(np.median(mag_sum_sub) + 1e-12)

    if med > 0:
        p2m = peak / med
    else:
        p2m = 0.0

    return {
        "fb_hz": fb,
        "bin_index": k,
        "freqs": freqs0,
        "X0": X0,
        "X1": X1,
        "mag0": mag0,
        "mag1": mag1,
        "mag0_sub": mag0_sub,
        "mag1_sub": mag1_sub,
        "mag_sum_sub": mag_sum_sub,
        "p2m": p2m,
    }


# -------------------------------------------------
# Pluto helpers
# -------------------------------------------------

def setup_pluto():
    print("Connecting to Pluto Plus at", URI)

    sdr = adi.ad9361(uri=URI)

    sdr.rx_enabled_channels = [0, 1]
    sdr.tx_enabled_channels = [0]

    sdr.sample_rate = int(FS)
    sdr.rx_rf_bandwidth = int(RF_BW)
    sdr.tx_rf_bandwidth = int(RF_BW)

    sdr.rx_lo = int(FC)
    sdr.tx_lo = int(FC)

    sdr.gain_control_mode_chan0 = "manual"
    sdr.gain_control_mode_chan1 = "manual"

    sdr.rx_hardwaregain_chan0 = float(RX_GAIN0)
    sdr.rx_hardwaregain_chan1 = float(RX_GAIN1)

    sdr.tx_hardwaregain_chan0 = float(TX_GAIN)

    sdr.rx_buffer_size = int(RX_BUF)
    sdr.tx_cyclic_buffer = True

    try:
        sdr._rxadc.set_kernel_buffers_count(1)
    except Exception:
        pass

    print("Pluto setup complete")

    return sdr


def measure_block(sdr):
    rx = sdr.rx()

    if isinstance(rx, (list, tuple)) and len(rx) == 2:
        return np.asarray(rx[0]), np.asarray(rx[1])

    rx = np.asarray(rx)

    if rx.ndim == 2 and rx.shape[0] == 2:
        return rx[0], rx[1]

    if rx.ndim == 2 and rx.shape[1] == 2:
        return rx[:, 0], rx[:, 1]

    raise RuntimeError(f"Unexpected RX shape: {rx.shape}")


def build_tx_frame():
    pre = make_preamble_tone(FS, f0=PRE_F0, N=PRE_N, amp=PRE_AMP)
    chirp = make_chirp(FS, T_CHIRP, B_SWEEP, amp=TX_AMP_CHIRP)

    guard0 = np.zeros(GUARD0_N, dtype=np.complex64)

    frame = np.concatenate([pre, chirp, guard0]).astype(np.complex64)
    tx_wave = np.tile(frame, TX_FRAME_REPEATS)

    return pre, chirp, frame, tx_wave


def start_tx(sdr, tx_wave):
    try:
        sdr.tx_destroy_buffer()
    except Exception:
        pass

    sdr.tx(tx_wave)
    time.sleep(0.1)


def acquire_accumulated_rx(sdr):
    for _ in range(DISCARD_RX_READS):
        _ = measure_block(sdr)

    acc0 = []
    acc1 = []

    for _ in range(ACCUM_READS):
        a0, a1 = measure_block(sdr)
        acc0.append(a0)
        acc1.append(a1)

    r0 = np.concatenate(acc0)
    r1 = np.concatenate(acc1)

    return r0, r1


# -------------------------------------------------
# USB serial helper
# -------------------------------------------------

def find_stm_port():
    preferred_ports = [
        "/dev/ttyACM1",
        "/dev/ttyACM0",
    ]
    
    for port in preferred_ports:
        if os.path.exists(port):
            print(f"Found STM serial port: {port}", flush=True)
            return port
            
    all_ports = glob.glob("/dev/ttyACM*")
    
    if len(all_ports) > 0:
        print(f"Found STM serial Port: {all_ports[0]}", flush=True)
        return all_ports[0]
        
    raise RuntimeError("No STM serial port found. checked /dev/ttyACM*")

def open_usb_serial():
    USB_PORT = find_stm_port ()
    print("Opening USB serial port:", USB_PORT)

    ser = serial.Serial(
        port=USB_PORT,
        baudrate=USB_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=0
    )

    # Many STM32 USB serial devices reset when the serial port opens.
    # This gives the STM32 time to come back online.
    time.sleep(2.0)

    print("USB serial ready")

    return ser


# -------------------------------------------------
# Angle estimation
# -------------------------------------------------

def estimate_angle_from_target_bin(X0, X1, bin_index, d_rx, fc):
    """
    Estimate target angle from RX phase difference at the detected target beat bin.
    """
    if bin_index < 0:
        return 0.0

    if bin_index >= len(X0):
        return 0.0

    if len(X0) != len(X1):
        return 0.0

    cross = X1[bin_index] * np.conj(X0[bin_index])
    delta_phi = np.angle(cross)

    wavelength = C / fc

    arg = delta_phi * wavelength / (2.0 * np.pi * d_rx)
    arg = np.clip(arg, -1.0, 1.0)

    theta_deg = np.degrees(np.arcsin(arg))

    return float(theta_deg)


# -------------------------------------------------
# Main processing
# -------------------------------------------------

def process_once(sdr, pre, chirp, frame, bg_state):
    Npre = len(pre)
    Nchirp = len(chirp)
    Nframe = len(frame)

    r0, r1 = acquire_accumulated_rx(sdr)

    start = earliest_strong_peak_start_frame_aligned(
        r0,
        pre,
        nframe=Nframe,
        thresh_ratio=PEAK_THRESH_RATIO,
        min_start=0
    )

    #if start is None:
        #return None

    max_frames = (min(r0.size, r1.size) - start) // Nframe
    use_frames = min(max_frames, CHIRPS_TO_AVG)

    #if use_frames < 4:
        #return None

    seg0 = r0[start:start + use_frames * Nframe].reshape(use_frames, Nframe)
    seg1 = r1[start:start + use_frames * Nframe].reshape(use_frames, Nframe)

    chirp0 = seg0[:, Npre:Npre + Nchirp]
    chirp1 = seg1[:, Npre:Npre + Nchirp]

    if CHIRP_EDGE_GUARD * 2 < Nchirp:
        chirp0 = chirp0[:, CHIRP_EDGE_GUARD:(Nchirp - CHIRP_EDGE_GUARD)]
        chirp1 = chirp1[:, CHIRP_EDGE_GUARD:(Nchirp - CHIRP_EDGE_GUARD)]
        ref = np.conjugate(chirp[CHIRP_EDGE_GUARD:(Nchirp - CHIRP_EDGE_GUARD)])
    else:
        ref = np.conjugate(chirp)

    beat0 = (chirp0 * ref[None, :]).reshape(-1)
    beat1 = (chirp1 * ref[None, :]).reshape(-1)

    det = detect_target_bin_with_bg(
        beat0,
        beat1,
        FS,
        FB_MIN_HZ,
        FB_MAX_HZ,
        bg_state["bg0"],
        bg_state["bg1"]
    )

    #if det is None:
        #return None

    # fb = det["fb_hz"]
    # p2m = det["p2m"]

    # range_m = two_way_range_from_fb(fb, B_SWEEP, T_CHIRP)

    # angle_deg = estimate_angle_from_target_bin(
        # det["X0"],
        # det["X1"],
        # det["bin_index"],
        # D_RX,
        # FC
    # )
    
    if det is None:
        range_m = 0.0
        angle_deg = 0.0
        quality = 0.0
        return range_m, angle_deg, quality
        
    if isinstance(det, (int, float, np.integer, np.floating)):
        fb = float(det)
        range_m = two_way_range_from_fb(fb, B_SWEEP, T_CHIRP)
        angle_deg = 0.0
        quality = 0.1
        return range_m, angle_deg, quality
        
    try:
        fb = float(det["fb_hz"])
    except Exeption:
        fb = 0.0
        
    try:
        range_m = two_way_range_from_fb(fb, B_SWEEP, T_CHIRP)
    except Exception:
        range_m = 0.0
        
    try:
        angle_deg = estimate_angle_from_target_bin(
            det["idx"],
            det["X0"],
            det["X1"],
            d=C/(2*FC),
            lam=C/FC,
        )
    except Exception:
        angle_deg = 0.0
        
    try:
        quality = float(det.get("quality", 1.0))
    except Exception:
        quality = 1.0
        
    #return range_m, angle_deg, quality

    q0 = coherence_metric(beat0)
    q1 = coherence_metric(beat1)
    q_avg = 0.5 * (q0 + q1)

    quality = 0.5 * min(p2m / 10.0, 1.0) + 0.5 * min(q_avg / 0.5, 1.0)
    quality = float(np.clip(quality, 0.0, 1.0))

    if p2m < MIN_PEAK_TO_MEDIAN or q_avg < MIN_COHERENCE:
        quality *= 0.25

    if bg_state["init_count"] < BG_INIT_FRAMES:
        bg_state["bg0"] = update_background(bg_state["bg0"], det["mag0"], 0.0)
        bg_state["bg1"] = update_background(bg_state["bg1"], det["mag1"], 0.0)
        bg_state["init_count"] += 1
    else:
        if quality < BG_FREEZE_QUALITY:
            bg_state["bg0"] = update_background(bg_state["bg0"], det["mag0"], BG_ALPHA)
            bg_state["bg1"] = update_background(bg_state["bg1"], det["mag1"], BG_ALPHA)
    
    if mag is not None and len(mag) > 0:
        print("mag len:", len(mag), "max:", np.max(mag), "min", np.min(mag), flush=True)
    else:
        print("mag is empty", flush=True)

    return range_m, angle_deg, quality


def make_ascii_packet(range_m, angle_deg, quality):
    """
    Packet sent to STM32.

    Format:
        range_m,angle_deg,quality\n

    Example:
        2.431,-7.52,0.81\n
    """
    return f"{range_m:.3f},{angle_deg:.2f},{quality:.2f}\n"


def main():
    print("Starting Pluto Plus FMCW radar on Raspberry Pi 3")
    print("Output mode: USB serial to STM32")
    print("Packet format: range_m,angle_deg,quality\\n")

    sdr = None
    ser = None

    try:
        sdr = setup_pluto()

        pre, chirp, frame, tx_wave = build_tx_frame()
        start_tx(sdr, tx_wave)

        ser = open_usb_serial()

        last_send = 0.0

        bg_state = {
            "bg0": None,
            "bg1": None,
            "init_count": 0,
        }

        while True:
            # result = process_once(sdr, pre, chirp, frame, bg_state)

            # if result is None:
                # continue

            # range_m, angle_deg, quality = result
            # msg = make_utf8_packet(range_m, angle_deg, quality)
            try:
                result = process_once(sdr, pre, chirp, frame, bg_state)
            except Exception as e:
                printf("Radar processing error:", repr(e), flush=True)
                result = (0.0, 0.0, 0.0)
                
            if result is None:
                result = (0.0, 0.0, 0.0)
                
            try:
                range_m, angle_deg, quality = result
            except Exception as e:
                print("Bad radar result format:", result, e, flush=True)
                range_m, angle_deg, quality = 0.0, 0.0, 0.0
                
            msg = make_ascii_packet(range_m, angle_deg, quality)
            
            now = time.time()
            
            if now - last_send >= MIN_SEND_PERIOD_S:
                try:
                    ser.write(msg.encode("ascii"))
                except serial.SerialTimeoutException:
                    print("Warning: USB serial write timeout", flush=True)
                except serial.SerialException as e:
                    print("USB serial error:", e, flush=True)
                    print.sleep(1.0)
                    
                if PRINT_PACKETS:
                    print("PI -> STM:", msg.strip(), flush=True)
                    
                last_send = now

            now = time.time()

            if now - last_send >= MIN_SEND_PERIOD_S:
                try:
                    ser.write(msg.encode("utf-8"))
                    print("PI -> STM:", msg.strip(), flush=True)
                except serial.SerialTimeoutException:
                    print("Warning: USB serial write timeout")
                except serial.SerialException as e:
                    print("USB serial error:", e)
                    time.sleep(1.0)

                if PRINT_PACKETS:
                    print(msg.strip())

                last_send = now

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        print("Cleaning up")

        if sdr is not None:
            try:
                sdr.tx_destroy_buffer()
            except Exception:
                pass

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        print("Done")


if __name__ == "__main__":
    main()
