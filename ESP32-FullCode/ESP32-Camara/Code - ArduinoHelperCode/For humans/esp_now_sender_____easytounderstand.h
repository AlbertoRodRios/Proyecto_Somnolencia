// This is the new file/tab: esp_now_sender.h

#ifndef ESP_NOW_SENDER_H
#define ESP_NOW_SENDER_H

#include <esp_now.h>
#include <WiFi.h>

// 1. Define the Data Structure
// This MUST be identical on both the Sender and Receiver
typedef struct packetData {
  float num1;
  float num2;
  float num3;
} packetData;

// Variable to store the peer info
esp_now_peer_info_t peerInfo;

// 2. The "Data Sent" Callback
// This function runs when ESP-NOW gets a status update
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 3. The Main Setup Function
// This is called once from your main .ino's setup()
void setup_esp_now_sender(uint8_t *peer_mac) {
  // Set ESP32 to Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the "data sent" callback
  esp_now_register_send_cb(OnDataSent);

  // Register the peer (the receiver)
  memcpy(peerInfo.peer_addr, peer_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

// 4. The "Send Data" Function
// This is called from your main .ino's loop()
void send_data_packet(const packetData *data) {
  esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *) data, sizeof(packetData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

#endif // ESP_NOW_SENDER_H