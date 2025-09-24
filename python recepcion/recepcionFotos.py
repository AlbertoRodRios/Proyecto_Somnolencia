import serial, struct, time, os

PORT = "COM4"        # <-- ajusta: COMx en Windows, /dev/ttyUSB0 en Linux, etc.
BAUD = 460800
TIMEOUT_S = 5.0
MAGIC = 0xAA55AA55
MAGIC_BYTES = struct.pack("<I", MAGIC)

def read_exact(ser, n):
    data = bytearray()
    deadline = time.time() + TIMEOUT_S
    while len(data) < n and time.time() < deadline:
        chunk = ser.read(n - len(data))
        if chunk:
            data.extend(chunk)
    if len(data) != n:
        raise TimeoutError(f"Timeout leyendo {n} bytes (recibidos {len(data)})")
    return bytes(data)

def find_magic(ser, max_ms=8000):
    """Escanea byte a byte hasta hallar MAGIC o detectar reinicio."""
    deadline = time.time() + (max_ms/1000)
    win = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        win += b
        if len(win) > 64:   # guarda últimas 64 bytes
            win = win[-64:]

        # --- Detector de reinicio ---
        txt = win.decode(errors="ignore")
        if "ets Jul" in txt or "rst:" in txt or "boot:" in txt:
            print("⚠️ ESP32 parece haberse reiniciado (texto de arranque detectado).")
            win.clear()
            # sigue esperando hasta que vuelva a enviar algo

        # --- Detector de MAGIC ---
        if len(win) >= 4 and win[-4:] == MAGIC_BYTES:
            return True
    return False

def request_capture(ser):
    ser.write(b'c')
    ser.flush()

def main():
    os.makedirs("capturas", exist_ok=True)
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        ser.setDTR(False); ser.setRTS(False)
        time.sleep(0.2)
        ser.reset_input_buffer(); ser.reset_output_buffer()

        print(f"Puerto {PORT} abierto @ {BAUD} bps")
        idx = 0
        while True:
            request_capture(ser)
            print("Esperando MAGIC o reinicio...")

            if not find_magic(ser, max_ms=10000):
                print("No se encontró MAGIC en 10 s. Reintentando...")
                continue

            # --- Llegó MAGIC: leer longitud ---
            raw_len = read_exact(ser, 4)
            length, = struct.unpack("<I", raw_len)
            if length == 0 or length > 2_000_000:
                print(f"Longitud inválida: {length}.")
                continue

            jpg = read_exact(ser, length)
            raw_sum = read_exact(ser, 2)
            checksum_recv, = struct.unpack("<H", raw_sum)
            checksum_calc = sum(jpg) & 0xFFFF

            if checksum_recv != checksum_calc:
                print(f"Checksum NO coincide (recv={checksum_recv:#06x}, calc={checksum_calc:#06x})")
                continue

            ts = time.strftime("%Y%m%d_%H%M%S")
            fname = f"capturas/foto_{ts}_{idx:04d}.jpg"
            with open(fname, "wb") as f:
                f.write(jpg)
            print(f"✅ Guardado: {fname} ({len(jpg)} bytes)")
            idx += 1
            time.sleep(0.5)

if __name__ == "__main__":
    main()
