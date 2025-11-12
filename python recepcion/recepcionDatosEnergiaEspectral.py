import serial, csv, os
import time

# --- Configuration ---
PORT = "COM8"
BAUDIOS = 115200 
FILENAME = "featuresSomnolencia.csv"
AWAKE = True
SUBJECT_ID = "S02"
SESSION_ID = "SES03"
EXPERIMENT_CONDITION = "normal"

# --- Umbrales de Detección ---
MAX_PTP_THRESHOLD = 20000 
MIN_ENERGY_FRAC_THRESHOLD = 0.20
# --- End Configuration ---

def get_initial_window_counter(filename):
    """Lee el último número de ventana del CSV para continuar la secuencia."""
    if not os.path.exists(filename) or os.path.getsize(filename) == 0:
        return 0
    try:
        with open(filename, 'r', newline='', encoding="utf-8") as f:
            rows = list(csv.reader(f))
            if len(rows) <= 1: return 0
            return int(rows[-1][6]) + 1
    except (IOError, IndexError, ValueError) as e:
        print(f"⚠️  Advertencia: No se pudo leer el último número de ventana de '{filename}'. Reiniciando a 0. Error: {e}")
        return 0

def write_headers(writer):
    """Escribe la fila de encabezado del CSV."""
    headers = [
        "subject_id", "session_id", "date", "time", "timestamp", "session_duration", 
        "window_sequence_num", "experiment_condition", "ax_rms", "ax_var", "ax_energy", 
        "ax_ptp", "ax_skew", "ax_kurt", "ax_mean", "ay_rms", "ay_var", "ay_energy", 
        "ay_ptp", "ay_skew", "ay_kurt", "ay_mean", "az_rms", "az_var", "az_energy", 
        "az_ptp", "az_skew", "az_kurt", "az_mean", "gx_rms", "gx_var", "gx_energy", 
        "gx_ptp", "gx_skew", "gx_kurt", "gx_mean", "gy_rms", "gy_var", "gy_energy", 
        "gy_ptp", "gy_skew", "gy_kurt", "gy_mean", "gz_rms", "gz_var", "gz_energy", 
        "gz_ptp", "gz_skew", "gz_kurt", "gz_mean", "xcorr_ax_ay", "xcorr_ax_az", 
        "xcorr_ay_az", "xcorr_gx_gy", "xcorr_gx_gz", "xcorr_gy_gz", "ppg_rms", 
        "ppg_var", "ppg_energy", "ppg_ptp", "ppg_skew", "ppg_kurt", "ppg_mean", 
        "ppg_B0_frac", "ppg_B1_frac", "ppg_B2_frac", "acc_L_frac", "acc_M_frac", 
        "gyro_L_frac", "gyro_M_frac", "awake"
    ]
    writer.writerow(headers)

def get_metadata(session_start_time, window_counter):
    """Genera los metadatos para la ventana de datos actual."""
    current_time_str = time.strftime("%H:%M:%S")
    date_str = time.strftime("%Y-%m-%d")
    return [
        SUBJECT_ID, SESSION_ID, date_str, current_time_str, time.time(),
        time.time() - session_start_time, window_counter, EXPERIMENT_CONDITION
    ]

def main_loop():
    esp32 = None
    while True:
        try:
            print(f"Intentando conectar al ESP32 en {PORT} a {BAUDIOS} bps...")
            esp32 = serial.Serial(PORT, BAUDIOS, timeout=2)
            time.sleep(2)
            esp32.flushInput()
            print("✅ ESP32 Conectado. Realizando handshake...")

            esp32.write(b"READY\n")
            
            response = esp32.readline().decode(errors="ignore").strip()
            if response == "READY":
                print("🤝 Handshake exitoso. Iniciando sesión de registro de datos.")
            else:
                print(f"Handshake no recibido, se obtuvo: '{response}'. Reintentando...")
                if esp32 and esp32.is_open: esp32.close()
                time.sleep(3)
                continue

            session_start_time = time.time()
            window_counter = get_initial_window_counter(FILENAME)
            
            file_exists = os.path.exists(FILENAME) and os.path.getsize(FILENAME) > 0
            action = 'a' if file_exists else 'w'

            with open(FILENAME, action, newline='', encoding="utf-8") as features_csv:
                writer = csv.writer(features_csv)
                if not file_exists:
                    write_headers(writer)

                while True:
                    line = esp32.readline().decode(errors="ignore").strip()
                    if not line: continue

                    data_points = line.split(',')
                    if len(data_points) == 62:
                        metadata = get_metadata(session_start_time, window_counter)
                        # 1 = despierto, 0 = dormido, modificar al momento de guardar muestras de uno u otro
                        awake_label = [f"{1 if AWAKE else 0}"]
                        full_row = metadata + data_points + awake_label
                        
                        writer.writerow(full_row)
                        features_csv.flush()

                        try:
                            ppg_ptp = float(full_row[51 + 8]) 
                            ppg_b1_frac = float(full_row[56 + 8])
                            # --- Fórmula de Estimación Final y Calibrada ---
                            if ppg_ptp > MAX_PTP_THRESHOLD:
                                pulse_icon = "⚪️"
                                bpm_display = "--- (Sensor libre)"
                            elif ppg_b1_frac > MIN_ENERGY_FRAC_THRESHOLD:
                                pulse_icon = "❤️"
                                # Fórmula ajustada: base de 45 BPM y un multiplicador más bajo (25).
                                # Esto mapeará una señal fuerte (como 0.7) a ~63 BPM.
                                activity_index = 45 + (ppg_b1_frac * 25) 
                                bpm_display = f"~{activity_index:.0f} BPM (est.)"
                            else:
                                pulse_icon = "🟡"
                                bpm_display = "--- (Buscando pulso...)"

                            print(f"\rRegistrando Ventana #{full_row[6]:<4} | {pulse_icon} {bpm_display:<25} | PPG PTP: {ppg_ptp:<8.2f}", end="")
                        
                        except (ValueError, IndexError) as e:
                            print(f"\rError al procesar la fila #{full_row[6]}: {e}", end="")
                        
                        window_counter += 1
                    else:
                        print(f"\n⚠️  Omitiendo línea (se esperaban 62 características, se recibieron {len(data_points)}): {line[:80]}...")
        
        except serial.SerialException as e:
            print(f"\n❌ Error Serial: {e}")
        except KeyboardInterrupt:
            print("\n\n🛑 Registro de datos detenido por el usuario.")
            break
        except Exception as e:
            print(f"\nOcurrió un error inesperado: {e}")
        finally:
            if esp32 and esp32.is_open:
                esp32.close()
                print("\nPuerto serial cerrado.")
            print("Reintentando conexión en 5 segundos...")
            time.sleep(5)

if __name__ == "__main__":
    main_loop()