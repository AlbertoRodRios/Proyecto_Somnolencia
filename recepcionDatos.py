import serial, csv
esp32 = serial.Serial("COM3", 115200,timeout=500)
with open("features.csv", 'w', newline='', encoding="utf-8") as features_csv:
    writer = csv.writer(features_csv)
    writer.writerow(["ax_rms", "ax_var", "ax_energy", "ax_ptp", "ax_skew","ax_kurt",
      "ay_rms", "ay_var", "ay_energy", "ay_ptp", "ay_skew", "ay_kurt",
      "az_rms", "az_var", "az_energy", "az_ptp", "az_skew", "az_kurt",
      "gx_rms", "gx_var", "gx_energy", "gx_ptp" "gx_skew", "gx_kurt",
      "gy_rms", "gy_var", "gy_energy", "gy_ptp", "gy_skew", "gy_kurt",
      "gz_rms", "gz_var", "gz_energy", "gz_ptp", "gz_skew", "gz_kurt",
      "xcorr_ax_ay", "xcorr_ax_az", "xcorr_ay_az", "xcorr_gx_gy", "xcorr_gx_gz", "xcorr_gy_gz"])  
    while True:
        if esp32.readline().decode(errors="ignore").strip() == "READY":
            break
    count = 0
    MAX_ROWS = 100
    while count < MAX_ROWS:
        line = esp32.readline().decode(errors="ignore").strip()
        if not line:
            continue
        print(f"Linea: {line}")
        writer.writerow(line.split(","))
        count += 1