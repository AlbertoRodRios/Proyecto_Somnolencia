// This is the new file/tab: esp_now_sender.h

#ifndef ESP_NOW_SENDER_H
#define ESP_NOW_SENDER_H

#include <esp_now.h>
#include <WiFi.h>

// 1. Define the Data Structure
// This MUST be identical on both the Sender and Receiver
typedef struct packetData {
    float features[62];
} packetData;

// Variable to store the peer info
esp_now_peer_info_t peerInfo;

// 2. The "Data Sent" Callback (UPDATED)
// This function runs when ESP-NOW gets a status update
//
// <<< THIS IS THE MAIN FIX >>>
// The function signature was changed to match the new ESP32 core library
//
// Old: void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// New: void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
//
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  
  // Note: The new callback signature doesn't provide the MAC address
  // directly, but it's not needed for this simple "success/fail" log.

  Serial.print("\n[CALLBACK] Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 3. The Main Setup Function (UPDATED)
// This is called once from your main .ino's setup()
void setup_esp_now_sender(uint8_t *peer_mac) {
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

  // Register the "data sent" callback
  // This line will now work
  esp_now_register_send_cb(OnDataSent);
  Serial.println("Send Callback Registered.");

  // Register the peer (the receiver)
  memcpy(peerInfo.peer_addr, peer_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Print the peer MAC address we are adding
  Serial.print("Adding Peer: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", peer_mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add peer");
    return;
  } else {
    Serial.println("Peer Added Successfully.");
  }
}

// 4. The "Send Data" Function (UPDATED)
// This is called from your main .ino's loop()
void send_data_packet(const packetData *data) {
  

  // Send the data
  esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *) data, sizeof(packetData));

  // Print the result of the *queuing* action
  if (result == ESP_OK) {
    Serial.println("  ...Packet Queued for sending (ESP_OK)");
  } else {
    Serial.println("  ...[ERROR] Failed to queue packet for sending!");
  }
  // The *actual* delivery status will arrive later in the OnDataSent callback
}

#endif // ESP_NOW_SENDER_H