#ifndef ESP_NOW_RECEIVER_H
#define ESP_NOW_RECEIVER_H

#include <esp_now.h>
#include <WiFi.h>

// 1. Define the Data Structure
// This MUST be identical on both the Transmitter and Receiver
typedef struct packetData {
  float Num1;
} packetData;

packetData receivedData;

volatile bool newDataAvailable = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  newDataAvailable = true;
}
void setup_esp_now_receiver() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
#endif 