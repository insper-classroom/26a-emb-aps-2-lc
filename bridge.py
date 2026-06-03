"""
bridge.py v2 — Echo test com 16 kHz, 12-bit, DC offset removido, normalização.

Uso:
    pip install pyserial numpy
    python bridge.py --port COM3
"""

import argparse
import struct
import sys
import time
import wave
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
from scipy import signal as scipy_signal


SAMPLE_RATE = 16000
RECORDINGS_DIR = Path("recordings")
PWM_WRAP = 4095          # mesmo do firmware
PWM_MID = 2048

# ---------- Parâmetros de filtragem ----------
VOICE_LOW_HZ  = 300       # corta graves abaixo da voz (ruído de fonte, hum)
VOICE_HIGH_HZ = 3400      # corta agudos acima da voz (chiado, hiss)
GATE_THRESHOLD = 0.05     # gate de ruído: silencia samples abaixo desse % do pico
GATE_ATTACK = 0.001       # constantes de tempo do gate em segundos
GATE_RELEASE = 0.05

# Marcadores
SEND_PLAY_START = b"<<P>"
SEND_PLAY_END   = b"<<X>"
RECV_REC_START  = b"<<S>"
RECV_REC_END    = b"<<E>"


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("Nenhuma porta serial encontrada.")
        return
    print("Portas disponíveis:")
    for p in ports:
        print(f"  {p.device}  —  {p.description}")


def parse_samples_12bit(raw_bytes: bytes) -> np.ndarray:
    """
    Converte bytes do firmware (little-endian, 2 bytes por sample, 12-bit) em ndarray uint16.
    """
    # Se número ímpar de bytes, descarta o último (incompleto)
    if len(raw_bytes) % 2 != 0:
        raw_bytes = raw_bytes[:-1]
    return np.frombuffer(raw_bytes, dtype="<u2")


def filter_voice(samples_centered: np.ndarray) -> np.ndarray:
    """
    Aplica passa-banda 300-3400Hz + gate de ruído num sinal float centrado em zero.
    Retorna float centrado em zero.
    """
    # 1. Passa-banda Butterworth ordem 4
    nyquist = SAMPLE_RATE / 2
    low = VOICE_LOW_HZ / nyquist
    high = VOICE_HIGH_HZ / nyquist
    b, a = scipy_signal.butter(4, [low, high], btype="band")
    filtered = scipy_signal.filtfilt(b, a, samples_centered)

    # 2. Noise gate com envelope follower
    peak = np.max(np.abs(filtered))
    if peak < 1.0:
        return filtered
    threshold = peak * GATE_THRESHOLD

    # Envelope da amplitude instantânea com attack/release
    abs_signal = np.abs(filtered)
    envelope = np.zeros_like(abs_signal)
    attack_coef = 1.0 - np.exp(-1.0 / (GATE_ATTACK * SAMPLE_RATE))
    release_coef = 1.0 - np.exp(-1.0 / (GATE_RELEASE * SAMPLE_RATE))

    env = 0.0
    for i in range(len(abs_signal)):
        if abs_signal[i] > env:
            env += (abs_signal[i] - env) * attack_coef
        else:
            env += (abs_signal[i] - env) * release_coef
        envelope[i] = env

    # Gain do gate: 1 quando envelope > threshold, 0 abaixo, transição suave
    gain = np.clip((envelope - threshold * 0.5) / (threshold * 0.5), 0.0, 1.0)
    return filtered * gain


def process_for_wav(samples_u12: np.ndarray) -> np.ndarray:
    """
    Remove DC, aplica filtro de voz, normaliza, devolve int16.
    """
    s = samples_u12.astype(np.float32)
    s -= s.mean()
    s = filter_voice(s)
    peak = np.max(np.abs(s))
    if peak > 1.0:
        s *= 32767.0 * 0.9 / peak
    return s.astype(np.int16)


def process_for_playback(samples_u12: np.ndarray) -> np.ndarray:
    """
    Remove DC, aplica filtro de voz, normaliza, retorna formato 12-bit centrado em PWM_MID.
    """
    s = samples_u12.astype(np.float32)
    s -= s.mean()
    s = filter_voice(s)
    peak = np.max(np.abs(s))
    if peak > 1.0:
        s *= (PWM_MID * 0.9) / peak
    s += PWM_MID
    s = np.clip(s, 0, PWM_WRAP)
    return s.astype(np.uint16)


def save_wav(samples_int16: np.ndarray, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)              # 16-bit
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(samples_int16.tobytes())
    duration = len(samples_int16) / SAMPLE_RATE
    print(f"  ↳ salvo: {path} ({len(samples_int16)} samples, {duration:.2f}s)")


def play_back(ser: serial.Serial, samples_u12: np.ndarray):
    print(f"  ↳ tocando de volta ({len(samples_u12)} samples)...")
    # Empacota em little-endian: 2 bytes por sample
    raw = samples_u12.astype("<u2").tobytes()

    ser.write(SEND_PLAY_START)
    chunk = 512
    for i in range(0, len(raw), chunk):
        ser.write(raw[i:i+chunk])
        # cada chunk de 512 bytes = 256 samples = 16ms @ 16kHz
        time.sleep((chunk // 2) / SAMPLE_RATE)
    ser.write(SEND_PLAY_END)
    ser.flush()
    print("  ↳ playback enviado")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="porta serial do Pico (ex: COM3)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if not args.port:
        list_ports()
        sys.exit("\nUse: python bridge.py --port <porta>")

    print(f"Abrindo {args.port}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()

    print("Conectado. Aperte o botão no Pico pra gravar.\n")

    state = "idle"
    buffer = bytearray()
    window = bytearray(b"\x00\x00\x00\x00")
    rec_idx = 1

    try:
        while True:
            data = ser.read(2048)
            if not data:
                continue

            for byte in data:
                window[0] = window[1]
                window[1] = window[2]
                window[2] = window[3]
                window[3] = byte

                if bytes(window) == RECV_REC_START:
                    if state == "idle":
                        print(f"[{rec_idx}] Gravando...")
                        buffer.clear()
                        state = "recording"
                elif bytes(window) == RECV_REC_END:
                    if state == "recording":
                        # Os últimos 3 bytes salvos no buffer eram parte do "<<E>"
                        if len(buffer) >= 3:
                            del buffer[-3:]

                        samples_u12 = parse_samples_12bit(bytes(buffer))
                        print(f"[{rec_idx}] Fim ({len(samples_u12)} samples)")

                        wav_path = RECORDINGS_DIR / f"rec_{rec_idx:03d}.wav"
                        save_wav(process_for_wav(samples_u12), wav_path)

                        play_back(ser, process_for_playback(samples_u12))

                        rec_idx += 1
                        state = "idle"
                elif state == "recording":
                    buffer.append(byte)

    except KeyboardInterrupt:
        print("\nFinalizado.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()