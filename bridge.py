"""
bridge.py v2 — Echo test com 16 kHz, 12-bit, DC offset removido, normalização.

Uso:
    pip install pyserial numpy
    python bridge.py --port COM3
"""

import argparse
import os
import struct
import sys
import time
import wave
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
from scipy import signal as scipy_signal

try:
    from openai import OpenAI          # só necessário no modo chat
except ImportError:
    OpenAI = None

try:
    from dotenv import load_dotenv     # carrega OPENAI_API_KEY do arquivo .env
    load_dotenv()
except ImportError:
    pass


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

# ---------- Pipeline de IA (Whisper -> GPT -> TTS) ----------
TRANSCRIBE_MODEL = "whisper-1"
CHAT_MODEL       = "gpt-4o-mini"
TTS_MODEL        = "gpt-4o-mini-tts"  # bem mais natural/humano que o tts-1
TTS_VOICE        = "coral"            # alloy, ash, ballad, coral, echo, fable, nova, onyx, sage, shimmer
TTS_PCM_RATE     = 24000              # TTS em pcm sai a 24 kHz, 16-bit, mono
TTS_INSTRUCTIONS = (                  # só o gpt-4o-mini-tts usa isto (steer de tom)
    "Fale em português do Brasil de forma natural, calorosa e conversacional, "
    "como uma pessoa real numa conversa — ritmo tranquilo e entonação expressiva."
)

SYSTEM_PROMPT = (
    "Você é um assistente de voz embarcado num controle físico. "
    "Responda em português do Brasil, de forma curta e direta — no máximo 2 ou 3 "
    "frases, sem listas nem markdown, porque a resposta será lida em voz alta."
)


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


def process_raw(samples_u12: np.ndarray) -> np.ndarray:
    """
    Só remove DC e normaliza — SEM filtro nenhum. Mostra o sinal cru do ADC,
    pra separar problema de captura (hardware/ADC) de problema de DSP/filtro.
    """
    s = samples_u12.astype(np.float32)
    s -= s.mean()
    peak = np.max(np.abs(s)) if len(s) else 0.0
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
        s *= (PWM_MID * 0.97) / peak
    s += PWM_MID
    s = np.clip(s, 0, PWM_WRAP)
    return s.astype(np.uint16)


def save_wav(samples_int16: np.ndarray, path: Path, rate: int = SAMPLE_RATE):
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)              # 16-bit
        wf.setframerate(rate)
        wf.writeframes(samples_int16.tobytes())
    duration = len(samples_int16) / rate
    print(f"  ↳ salvo: {path} ({len(samples_int16)} samples, {duration:.2f}s @ {rate} Hz)")


PLAY_PRIME_SAMPLES = 3072    # cushion pré-enviado antes de marcar o tempo (~192 ms)
PLAY_CHUNK_SAMPLES = 512     # 1 KB por write (~32 ms). Ring do firmware = 8192.


def play_back(ser: serial.Serial, samples_u12: np.ndarray):
    """
    Toca no speaker sem picotar. O ring do firmware (~0.5 s) seca se a gente apenas
    der sleeps fixos por chunk — no Windows o time.sleep tem granularidade de ~15 ms
    e atrasa os envios, esvaziando o ring (underrun = "picotado").

    Solução: (1) pré-enche um 'prime' de samples como cushion e (2) envia o resto
    ancorado em tempo ABSOLUTO (perf_counter), então a jitter do sleep não acumula
    e o ring fica sempre ~PRIME cheio.
    """
    n = len(samples_u12)
    print(f"  ↳ tocando de volta ({n} samples, {n / SAMPLE_RATE:.1f}s)...")
    raw = samples_u12.astype("<u2").tobytes()

    ser.write(SEND_PLAY_START)

    # 1) pré-enche o ring antes de começar a contar o tempo
    ser.write(raw[:PLAY_PRIME_SAMPLES * 2])

    # 2) envia o resto em ritmo real, ancorado em t0
    t0 = time.perf_counter()
    sent = PLAY_PRIME_SAMPLES
    pos = PLAY_PRIME_SAMPLES * 2
    while pos < len(raw):
        block = raw[pos:pos + PLAY_CHUNK_SAMPLES * 2]
        ser.write(block)
        pos += len(block)
        sent += len(block) // 2
        # espera até o instante em que (sent - PRIME) samples já deveriam ter tocado
        target = t0 + (sent - PLAY_PRIME_SAMPLES) / SAMPLE_RATE
        dt = target - time.perf_counter()
        if dt > 0:
            time.sleep(dt)

    ser.write(SEND_PLAY_END)
    ser.flush()
    print("  ↳ playback enviado")


def transcribe(client: "OpenAI", wav_path: Path) -> str:
    """Whisper: WAV -> texto (pt)."""
    with open(wav_path, "rb") as f:
        result = client.audio.transcriptions.create(
            model=TRANSCRIBE_MODEL, file=f, language="pt", response_format="text",
        )
    return result.strip() if isinstance(result, str) else result.text.strip()


def chat_reply(client: "OpenAI", history: list, user_text: str) -> str:
    """GPT: mantém o histórico da conversa e devolve a resposta."""
    history.append({"role": "user", "content": user_text})
    resp = client.chat.completions.create(model=CHAT_MODEL, messages=history)
    answer = resp.choices[0].message.content.strip()
    history.append({"role": "assistant", "content": answer})
    return answer


def synthesize_to_u12(client: "OpenAI", text: str, voice: str) -> np.ndarray:
    """TTS -> samples 12-bit (0..PWM_WRAP) a 16 kHz, prontos pro play_back()."""
    with client.audio.speech.with_streaming_response.create(
        model=TTS_MODEL, voice=voice, input=text, response_format="pcm",
        instructions=TTS_INSTRUCTIONS,
    ) as resp:
        pcm = resp.read()                       # int16 LE mono @ 24 kHz

    audio = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    # 24 kHz -> 16 kHz (mesma taxa que o firmware espera)
    audio = scipy_signal.resample_poly(audio, SAMPLE_RATE, TTS_PCM_RATE)
    # int16 centrado em 0 -> 12-bit centrado em PWM_MID
    peak = float(np.max(np.abs(audio))) if len(audio) else 0.0
    if peak > 1.0:
        audio *= (PWM_MID * 0.97) / peak
    audio += PWM_MID
    audio = np.clip(audio, 0, PWM_WRAP)
    return audio.astype(np.uint16)


def run_chat_pipeline(client: "OpenAI", history: list, wav_path: Path,
                      ser: serial.Serial, voice: str):
    """Whisper -> GPT -> TTS -> speaker. Erros em qualquer etapa abortam só o turno."""
    try:
        text = transcribe(client, wav_path)
    except Exception as e:
        print(f"  ↳ ERRO transcrição: {e}")
        return
    print(f"  🗣  você: {text!r}")
    if not text.strip():
        print("  ↳ (transcrição vazia, ignorando)")
        return

    try:
        answer = chat_reply(client, history, text)
    except Exception as e:
        print(f"  ↳ ERRO GPT: {e}")
        return
    print(f"  🤖 GPT: {answer!r}")

    try:
        u12 = synthesize_to_u12(client, answer, voice)
    except Exception as e:
        print(f"  ↳ ERRO TTS: {e}")
        return
    play_back(ser, u12)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="porta serial do Pico (ex: COM3)")
    ap.add_argument("--baud", type=int, default=460800,
                    help="deve casar com PICO_DEFAULT_UART_BAUD_RATE do firmware (460800)")
    ap.add_argument("--mode", choices=["chat", "echo"], default="chat",
                    help="chat = Whisper→GPT→TTS (padrão); echo = toca o próprio áudio")
    ap.add_argument("--voice", default=TTS_VOICE,
                    help="voz do TTS (alloy, echo, fable, onyx, nova, shimmer)")
    ap.add_argument("--no-play", action="store_true",
                    help="no modo echo, não reproduz — teste só de captura")
    args = ap.parse_args()

    # No Windows o time.sleep tem granularidade de ~15 ms por padrão, o que faz o
    # pacing do playback engasgar. Eleva a resolução do timer pra ~1 ms.
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    if not args.port:
        list_ports()
        sys.exit("\nUse: python bridge.py --port <porta>")

    client = None
    history = None
    if args.mode == "chat":
        if OpenAI is None:
            sys.exit("Modo chat precisa do pacote openai:  pip install openai")
        if not os.getenv("OPENAI_API_KEY"):
            sys.exit('Defina OPENAI_API_KEY (PowerShell: $env:OPENAI_API_KEY="sk-...").')
        client = OpenAI()
        history = [{"role": "system", "content": SYSTEM_PROMPT}]

    print(f"Abrindo {args.port}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()

    if args.mode == "chat":
        print(f"Modo CHAT — Whisper → {CHAT_MODEL} → TTS({args.voice}).")
    else:
        print("Modo ECHO — toca de volta o próprio áudio.")
    print("Conectado. Aperte o botão no Pico pra gravar.\n")

    state = "idle"
    buffer = bytearray()
    window = bytearray(b"\x00\x00\x00\x00")
    rec_idx = 1
    t_start = 0.0

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
                        t_start = time.time()
                        state = "recording"
                elif bytes(window) == RECV_REC_END:
                    if state == "recording":
                        elapsed = time.time() - t_start
                        # Os últimos 3 bytes salvos no buffer eram parte do "<<E>"
                        if len(buffer) >= 3:
                            del buffer[-3:]

                        raw_bytes = bytes(buffer)
                        samples_u12 = parse_samples_12bit(raw_bytes)
                        n = len(samples_u12)
                        eff_rate = n / elapsed if elapsed > 0 else 0.0

                        # ---- Diagnóstico de captura ----
                        if n:
                            dc = float(samples_u12.mean())
                            smin, smax = int(samples_u12.min()), int(samples_u12.max())
                        else:
                            dc, smin, smax = 0.0, 0, 0
                        print(f"[{rec_idx}] Fim: {n} samples em {elapsed:.2f}s")
                        print(f"      taxa real  ≈ {eff_rate:.0f} Hz   (firmware deveria dar {SAMPLE_RATE})")
                        print(f"      ADC dc={dc:.0f} (~{dc/4095*3.3:.2f} V)  min={smin}  max={smax}  span={smax-smin}")

                        # ---- Análise de alinhamento ----
                        # No formato certo, todo byte MSB (índice ímpar) é <= 0x0F (12-bit).
                        def bad_msb_frac(bs, off):
                            m = len(bs) - off
                            arr = np.frombuffer(bs[off:off + 2 * (m // 2)], dtype=np.uint8)
                            msb = arr[1::2]
                            return (msb > 0x0F).mean() * 100 if len(msb) else 100.0
                        print(f"      bytes[0:24] = {raw_bytes[:24].hex(' ')}")
                        print(f"      len(buffer)={len(raw_bytes)} (par? {len(raw_bytes) % 2 == 0})")
                        print(f"      MSB>0x0F:  alinh.0 = {bad_msb_frac(raw_bytes, 0):.1f}%   "
                              f"alinh.+1 = {bad_msb_frac(raw_bytes, 1):.1f}%")
                        if smax - smin < 50:
                            print("      ⚠ span baixíssimo — ADC quase parado (mic mudo / pino errado / sem sinal)")
                        if smin == 0 and smax >= 4094:
                            print("      ⚠ saturando 0..4095 — clipping / ganho alto demais / pino flutuando")

                        # WAV CRU (sem filtro) na taxa REAL medida -> pitch correto, mostra o ADC puro
                        raw_rate = int(round(eff_rate)) if eff_rate > 1000 else SAMPLE_RATE
                        save_wav(process_raw(samples_u12),
                                 RECORDINGS_DIR / f"raw_{rec_idx:03d}.wav", rate=raw_rate)
                        # WAV filtrado (como antes), na taxa nominal
                        save_wav(process_for_wav(samples_u12),
                                 RECORDINGS_DIR / f"rec_{rec_idx:03d}.wav")

                        if args.mode == "chat":
                            run_chat_pipeline(
                                client, history,
                                RECORDINGS_DIR / f"rec_{rec_idx:03d}.wav",
                                ser, args.voice)
                        elif not args.no_play:
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