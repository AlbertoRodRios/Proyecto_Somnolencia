import serial, csv, os
def writeHeaders():
    if not file_exists:
        headers = [
            "ax_rms", "ax_var", "ax_energy", "ax_ptp", "ax_skew", "ax_kurt",
            "ay_rms", "ay_var", "ay_energy", "ay_ptp", "ay_skew", "ay_kurt",
            "az_rms", "az_var", "az_energy", "az_ptp", "az_skew", "az_kurt",
            "gx_rms", "gx_var", "gx_energy", "gx_ptp", "gx_skew", "gx_kurt",
            "gy_rms", "gy_var", "gy_energy", "gy_ptp", "gy_skew", "gy_kurt",
            "gz_rms", "gz_var", "gz_energy", "gz_ptp", "gz_skew", "gz_kurt",
            "xcorr_ax_ay", "xcorr_ax_az","xcorr_ay_az","xcorr_gx_gy","xcorr_gx_gz","xcorr_gy_gz",
            "ppg_rms", "ppg_var", "ppg_energy", "ppg_ptp", "ppg_skew", "ppg_kurt",
            "ppg_B0_frac", "ppg_B1_frac", "ppg_B2_frac", "acc_L_frac", "acc_M_frac", "gyro_L_frac","gyro_M_frac",
            "awake"]
        writer.writerow(headers)
 
def getRowsFromTime(seconds):
    if seconds < 2:
        return 0
    return (seconds - 1) 

awake = True  # Cambiar a False si se quiere etiquetar como somnoliento
port =  "COM3"  # Cambiar al puerto correcto si es necesario      
filename = "featuresSomnolencia.csv"
file_exists = os.path.exists(filename)
action = 'a' if file_exists else 'w'

esp32 = serial.Serial(port, 115200,timeout=500)
with open(filename, action, newline='', encoding="utf-8") as features_csv:
    writer = csv.writer(features_csv)
    writeHeaders()
    esp32.write(str("READY\n").encode())
    while True:
        if esp32.readline().decode(errors="ignore").strip() == "READY":
            break
    count = 0
    MAX_ROWS = getRowsFromTime(300) #Limitar a N filas según tiempo en segundos
    while count < MAX_ROWS:
        line = esp32.readline().decode(errors="ignore").strip()
        line += ","
        line += "1" if awake else "0"
        if not line:
            continue
        print(f"{count}: {line}")
        writer.writerow(line.split(","))
        count += 1
        
        
