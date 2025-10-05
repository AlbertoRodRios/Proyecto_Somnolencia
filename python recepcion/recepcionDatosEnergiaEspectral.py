import serial, csv, os
import time

# Modificar estos parámetros según sea necesario
# Configuración inicial del puerto serial y archivo CSV
PORT =  "COM3"  # Cambiar al puerto correcto si es necesario  
BAUDIOS = 115200 # Velocidad de comunicación 
FILENAME = "featuresSomnolencia.csv" # Nombre del archivo CSV

# Configuración del estado (despierto o somnoliento)
AWAKE = True  # Cambiar a False si se quiere etiquetar como somnoliento

# Metadata del experimento (ajustar según sea necesario)
SUBJECT_ID = "S01"  # Cambiar por el ID real
SESSION_ID = "SES01"  # Identificador de sesión
EXPERIMENT_CONDITION = "normal"  # Condición experimental

# Inicio de la sesión y contador de ventanas (no modificar)
session_start_time = time.time() 
window_counter = 0

# Obtener fecha y hora actuales (no modificar)
date_str = time.strftime("%Y-%m-%d")  # Fecha
time_str = time.strftime("%H:%M:%S")  # Hora

# Verificar si el archivo ya existe para decidir el modo de apertura
file_exists = os.path.exists(FILENAME)
action = 'a' if file_exists else 'w'


# Función para escribir los encabezados en el archivo CSV
def writeHeaders():
    if not file_exists or os.path.getsize(FILENAME) == 0:
        headers = [
            # Metadata
            "subject_id", "session_id", "date", "time", "timestamp", 
            "session_duration", "window_sequence_num", "experiment_condition",
            # Features
            "ax_rms", "ax_var", "ax_energy", "ax_ptp", "ax_skew", "ax_kurt", "ax_mean",
            "ay_rms", "ay_var", "ay_energy", "ay_ptp", "ay_skew", "ay_kurt", "ay_mean",
            "az_rms", "az_var", "az_energy", "az_ptp", "az_skew", "az_kurt", "az_mean",
            "gx_rms", "gx_var", "gx_energy", "gx_ptp", "gx_skew", "gx_kurt", "gx_mean",
            "gy_rms", "gy_var", "gy_energy", "gy_ptp", "gy_skew", "gy_kurt", "gy_mean",
            "gz_rms", "gz_var", "gz_energy", "gz_ptp", "gz_skew", "gz_kurt", "gz_mean",
            "xcorr_ax_ay", "xcorr_ax_az","xcorr_ay_az","xcorr_gx_gy","xcorr_gx_gz","xcorr_gy_gz",
            "ppg_rms", "ppg_var", "ppg_energy", "ppg_ptp", "ppg_skew", "ppg_kurt", "ppg_mean",
            # Spectral features
            "ppg_B0_frac", "ppg_B1_frac", "ppg_B2_frac", "acc_L_frac", "acc_M_frac", "gyro_L_frac","gyro_M_frac",
            # Label
            "awake"]
        writer.writerow(headers)
 
# Función para calcular el número de filas según el tiempo en segundos
def getRowsFromTime(seconds):
    if seconds < 2:
        return 0
    return (seconds - 1) 

# Función para obtener los metadatos
def get_metadata():
    global window_counter
    metadata = [
        SUBJECT_ID,
        SESSION_ID,
        date_str,
        time_str,
        time.time(),  # timestamp
        time.time() - session_start_time,  # session_duration
        window_counter,  # window_sequence_num
        EXPERIMENT_CONDITION # experiment_condition
    ]
    window_counter += 1
    return metadata

# Abrir el puerto serial y el archivo CSV
esp32 = serial.Serial(PORT, BAUDIOS, timeout=0.5)
with open(FILENAME, action, newline='', encoding="utf-8") as features_csv:
    writer = csv.writer(features_csv)
    writeHeaders() 
    # Esperar a que el ESP32 esté listo
    esp32.write(str("READY\n").encode())
    while True:
        if esp32.readline().decode(errors="ignore").strip() == "READY":
            break
        time.sleep(0.1) 
    count = 0
    # Calcular el número máximo de filas según el tiempo deseado
    MAX_ROWS = getRowsFromTime(300) #Limitar a N filas según tiempo en segundos
    while count < MAX_ROWS:
        # Leer línea del serial y añadir etiqueta
        line = esp32.readline().decode(errors="ignore").strip()
        # Si la línea está vacía, continuar
        if not line:
            continue
        line += ","
        line += "1" if AWAKE else "0"
        # Añadir metadatos al inicio de cada línea
        line = ",".join(map(str, get_metadata())) + "," + line
        # Escribir la línea en pantalla y en el archivo CSV
        print(f"{count}: {line}")
        writer.writerow(line.split(","))
        count += 1
        
        
