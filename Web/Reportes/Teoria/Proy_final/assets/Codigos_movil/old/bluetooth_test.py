#!/usr/bin/env python3
import socket
import struct
import sys

# Dirección MAC y canal RFCOMM del ESP32
ESP32_ADDR = "88:13:BF:70:40:72"
PORT       = 1

def main():
    # Crear socket Bluetooth RFCOMM puro
    sock = socket.socket(
        socket.AF_BLUETOOTH,
        socket.SOCK_STREAM,
        socket.BTPROTO_RFCOMM
    )

    print(f"Conectando a {ESP32_ADDR}:{PORT} …")
    try:
        sock.connect((ESP32_ADDR, PORT))
    except OSError as e:
        print("❌ Error al conectar:", e)
        sys.exit(1)
    print("✅ Conectado. Envía valores −255…255 (o 'q' para salir).")

    try:
        while True:
            # Leer velocidad del Motor Derecho (B)
            txt = input("Velocidad MOTOR D (–255 a 255, q=salir): ").strip()
            if txt.lower() == 'q':
                break
            try:
                wr = int(txt)
            except ValueError:
                print("⚠️  Entrada inválida. Debe ser un número entre –255 y 255.")
                continue

            # Leer velocidad del Motor Izquierdo (A)
            txt2 = input("Velocidad MOTOR I (–255 a 255): ").strip()
            try:
                wl = int(txt2)
            except ValueError:
                print("⚠️  Entrada inválida. Debe ser un número entre –255 y 255.")
                continue

            # Empaquetar el paquete: encabezado 'H' + wr + wl (little-endian int16)
            packet = struct.pack('<c hh', b'H', wr, wl)
            try:
                sock.send(packet)
            except OSError as e:
                print("❌ Error enviando datos:", e)
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("🔌 Cerrando conexión…")
        sock.close()

if __name__ == "__main__":
    main()
