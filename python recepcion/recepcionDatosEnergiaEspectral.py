import serial, csv, os
import time

# --- Configuration ---
PORT = "COM8"
BAUDIOS = 921600
FILENAME = "featuresSomnolencia.csv"
AWAKE = True
SUBJECT_ID = "S01"
SESSION_ID = "SES01"
EXPERIMENT_CONDITION = "normal"
# --- End Configuration ---

session_start_time = time.time()
window_counter = 0
date_str = time.strftime("%Y-%m-%d")
time_str = time.strftime("%H:%M:%S")

file_exists = os.path.exists(FILENAME)
action = 'a' if file_exists else 'w'

def writeHeaders(writer):
    if not file_exists or os.path.getsize(FILENAME) == 0:
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

def get_metadata():
    global window_counter
    metadata = [
        SUBJECT_ID, SESSION_ID, date_str, time_str, time.time(),
        time.time() - session_start_time, window_counter, EXPERIMENT_CONDITION
    ]
    window_counter += 1
    return metadata

try:
    esp32 = serial.Serial(PORT, BAUDIOS, timeout=1)
    
    # Give the ESP32 ~2 seconds to initialize after the serial connection is made
    print("Connecting to ESP32...")
    time.sleep(2) 
    print("ESP32 Connected. Starting data logging.")

    with open(FILENAME, action, newline='', encoding="utf-8") as features_csv:
        writer = csv.writer(features_csv)
        writeHeaders(writer)

        # --- Handshake with ESP32 ---
        esp32.write(b"READY\n")
        while True:
            response = esp32.readline().decode(errors="ignore").strip()
            if response == "READY":
                print("Handshake successful. Receiving data...")
                break
            else:
                # This will print any unexpected messages from the ESP32 during handshake
                if response:
                    print(f"Waiting for READY, received: {response}")
            time.sleep(0.1)

        count = 0
        # Set a number of rows to capture, e.g., 20 windows
        MAX_ROWS = 20 

        while count < MAX_ROWS:
            line = esp32.readline().decode(errors="ignore").strip()

            if not line:
                continue
            
            # Validate that the line contains the correct number of data points
            if len(line.split(',')) == 62:
                line += f",{1 if AWAKE else 0}"
                metadata_str = ",".join(map(str, get_metadata()))
                full_line = f"{metadata_str},{line}"
                
                print(f"Row {count}: {full_line}")
                writer.writerow(full_line.split(","))
                features_csv.flush()  # Force data to be written to the file
                count += 1

except serial.SerialException as e:
    print(f"Error: Could not open port {PORT}. Please check the port and try again.")
    print(e)
except KeyboardInterrupt:
    print("\nData logging stopped by user.")
finally:
    if 'esp32' in locals() and esp32.is_open:
        esp32.close()
        print("Serial port closed.")