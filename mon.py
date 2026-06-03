"""
mon.py — monitor cru da serial pra diagnosticar o botão/UART.
Uso:  python mon.py [porta] [baud]      (default COM3 921600)
Abra, e durante os 12s aperte/solte o botão algumas vezes.
"""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

s = serial.Serial(port, baud, timeout=0.2)
print(f"Ouvindo {port} @ {baud}.")
print(">>> SEGURE o botao PRESSIONADO os 12s inteiros (nao solte) <<<")
total = bytearray()
t0 = time.time()
try:
    while time.time() - t0 < 12:
        d = s.read(512)
        if d:
            total += d
            print(f"  RX {len(d):4d} bytes | hex: {d[:24].hex(' ')} | ascii: {d[:24].decode('latin1')!r}")
except KeyboardInterrupt:
    print("(cancelado)")
finally:
    s.close()

print(f"\nTotal recebido: {len(total)} bytes")
if not total:
    print("=> NADA chegou. Botao nao detectado, fio do botao (GP3), ou TX/UART nao saindo.")
else:
    has_S = b"<<S>" in total
    has_E = b"<<E>" in total
    print(f"=> Marcadores: <<S>={has_S}  <<E>={has_E}")
    if has_S or has_E:
        print("   Baud OK! UART limpa. (Se o bridge nao via, o problema e no bridge.)")
    else:
        # heuristica: muitos bytes com bit alto setado sugere baud errada
        hi = sum(1 for b in total if b & 0x80) / len(total)
        print(f"   Sem marcadores. {hi*100:.0f}% dos bytes com bit alto.")
        print("   Se veio lixo/sem marcador => Debug Probe nao casou a baud 921600.")
