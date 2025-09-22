import serial, csv, os

def writeHeaders():
    if not file_exists:
        headers = [
            'ax_mean', 'ay_mean', 'az_mean', 'ax_std', 'ay_std', 'az_std',
            'ax_max', 'ay_max', 'az_max', 'ax_min', 'ay_min', 'az_min',
            'gx_mean', 'gy_mean', 'gz_mean', 'gx_std', 'gy_std', 'gz_std',
            'gx_max', 'gy_max', 'gz_max', 'gx_min', 'gy_min', 'gz_min',
            'ax_energy', 'ay_energy', 'az_energy',
            'gx_energy', 'gy_energy', 'gz_energy',
            'ax_rms', 'ay_rms', 'az_rms',
            'gx_rms', 'gy_rms', 'gz_rms',
            'xcorr_ax_gx', 'xcorr_ax_gy', 'xcorr_ax_gz',
            'xcorr_ay_gx', 'xcorr_ay_gy', 'xcorr_ay_gz',
            'xcorr_az_gx', 'xcorr_az_gy', 'xcorr_az_gz'
        ]
        writer.writerow(headers)
        
filename = "features.csv"
file_exists = os.path.exists(filename)
action = 'a' if file_exists else 'w'

esp32 = serial.Serial("COM3", 921600,timeout=500)
with open(filename, action, newline='', encoding="utf-8") as features_csv:
    writer = csv.writer(features_csv)
    writeHeaders()
    esp32.write(str("READY\n").encode())
    while True:
        if esp32.readline().decode(errors="ignore").strip() == "READY":
            break
    count = 0
    MAX_ROWS = 10
    while count < MAX_ROWS:
        line = esp32.readline().decode(errors="ignore").strip()
        if not line:
            continue
        print(f"{count}: {line}")
        writer.writerow(line.split(","))
        count += 1
        
        
