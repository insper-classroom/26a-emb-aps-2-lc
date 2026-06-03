"""
test_whisper.py — Testa transcrição dos WAVs gravados via OpenAI Whisper.

Uso:
    pip install openai
    set OPENAI_API_KEY=sk-...   (Windows PowerShell: $env:OPENAI_API_KEY="sk-...")
    python test_whisper.py                    # transcreve todos os WAVs em recordings/
    python test_whisper.py recordings/rec_005.wav   # ou um específico
"""

import os
import sys
from pathlib import Path

from openai import OpenAI

RECORDINGS_DIR = Path("recordings")


def transcribe(client: OpenAI, wav_path: Path) -> str:
    with open(wav_path, "rb") as f:
        result = client.audio.transcriptions.create(
            model="whisper-1",
            file=f,
            language="pt",     # força português
            response_format="text",
        )
    return result.strip() if isinstance(result, str) else result.text.strip()


def main():
    if not os.getenv("OPENAI_API_KEY"):
        sys.exit("Defina a variável OPENAI_API_KEY antes de rodar.")

    client = OpenAI()

    # Coleta os arquivos a transcrever
    if len(sys.argv) > 1:
        files = [Path(p) for p in sys.argv[1:]]
    else:
        files = sorted(RECORDINGS_DIR.glob("rec_*.wav"))
        if not files:
            sys.exit(f"Nenhum WAV encontrado em {RECORDINGS_DIR}/")

    print(f"Transcrevendo {len(files)} arquivo(s)...\n")

    for f in files:
        if not f.exists():
            print(f"  [skip] {f} (não existe)")
            continue
        try:
            text = transcribe(client, f)
            print(f"  {f.name:25s} → {text!r}")
        except Exception as e:
            print(f"  {f.name:25s} → ERRO: {e}")


if __name__ == "__main__":
    main()