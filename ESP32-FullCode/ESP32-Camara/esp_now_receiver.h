#ifndef ESP_NOW_RECEIVER_H
#define ESP_NOW_RECEIVER_H

#include <esp_now.h>
#include <WiFi.h>

// 1. Define the Data Structure (ACTUALIZADO)
// This MUST be identical on both the Sender and Receiver
typedef struct packetData {
  float features[62];
} packetData;

// 2. Create Global Variables
packetData receivedData;
volatile bool newDataAvailable = false;


// 3. The "Data Received" Callback
void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  
  const uint8_t* mac = info->src_addr;

  // --- FEEDBACK FOR DEBUGGING ---
  Serial.print("\n[CALLBACK] Packet received! ");
  Serial.print("Bytes: ");
  Serial.println(len);
  
  // Print the MAC address of the sender
  Serial.print("  From: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Check if the packet size matches our struct
  if (len == sizeof(packetData)) {
    // Copy the incoming data into our global 'receivedData' structure
    memcpy(&receivedData, incomingData, sizeof(packetData));
    
    // Set the flag so the main loop knows there's new data
    newDataAvailable = true;
    Serial.println("  Data copied & flag set.");
    
  } else {
    Serial.println("  [ERROR] Packet size mismatch!");
    Serial.print("  Expected: ");
    Serial.print(sizeof(packetData)); // Informará el nuevo tamaño
    Serial.print(" but received: ");
    Serial.println(len);
  }
}

// 4. The Main Setup Function
void setup_esp_now_receiver() {
  // Set ESP32 to Station mode
  WiFi.mode(WIFI_STA);
  Serial.println("Wi-Fi Mode set to Station.");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[FATAL] Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW Initialized.");
  }

  // Register the "data received" callback
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receive Callback Registered.");
}

#endif // ESP_NOW_RECEIVER_H