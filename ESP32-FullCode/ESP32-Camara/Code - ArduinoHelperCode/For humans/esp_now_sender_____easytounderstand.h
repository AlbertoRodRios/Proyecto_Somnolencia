#ifndef ESP_NOW_SENDER_H
#define ESP_NOW_SENDER_H

#include <esp_now.h>
#include <WiFi.h>

typedef struct packetData {
  float dormido;
} packetData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup_esp_now_sender(uint8_t *peer_mac) {
  WiFi.mode(WIFI_STA);
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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
void send_data_packet(const packetData *data) {
  esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *) data, sizeof(packetData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

#endif // ESP_NOW_SENDER_H