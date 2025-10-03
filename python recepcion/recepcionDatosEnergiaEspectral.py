import serial, csv, os
import time
# Función para escribir los encabezados en el archivo CSV
def writeHeaders():
    if not file_exists:
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
        subject_id,
        session_id,
        date_str,
        time_str,
        time.time(),  # timestamp
        time.time() - session_start_time,  # session_duration
        window_counter,  # window_sequence_num
        experiment_condition # experiment_condition
    ]
    window_counter += 1
    return metadata
# Modificar estos parámetros según sea necesario
# Configuración inicial del puerto serial y archivo CSV
port =  "COM3"  # Cambiar al puerto correcto si es necesario  
badios = 115200 # Velocidad de comunicación 
filename = "featuresSomnolencia.csv" # Nombre del archivo CSV

# Configuración del estado (despierto o somnoliento)
awake = True  # Cambiar a False si se quiere etiquetar como somnoliento

# Metadata del experimento (ajustar según sea necesario)
subject_id = "S01"  # Cambiar por el ID real
session_id = "SES01"  # Identificador de sesión
experiment_condition = "normal"  # Condición experimental

# Inicio de la sesión y contador de ventanas (no modificar)
session_start_time = time.time() 
window_counter = 0

# Obtener fecha y hora actuales (no modificar)
date_str = time.strftime("%Y-%m-%d")  # Fecha
time_str = time.strftime("%H:%M:%S")  # Hora

# Verificar si el archivo ya existe para decidir el modo de apertura
file_exists = os.path.exists(filename)
action = 'a' if file_exists else 'w'

# Abrir el puerto serial y el archivo CSV
esp32 = serial.Serial(port, badios, timeout=0.5)
with open(filename, action, newline='', encoding="utf-8") as features_csv:
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
        line += ","
        line += "1" if awake else "0"
        # Añadir metadatos al inicio de cada línea
        line = ",".join(map(str, get_metadata())) + "," + line
        # Si la línea está vacía, continuar
        if not line:
            continue
        # Escribir la línea en pantalla y en el archivo CSV
        print(f"{count}: {line}")
        writer.writerow(line.split(","))
        count += 1
        
        
