// This is the new file/tab: esp_now_receiver.h

#ifndef ESP_NOW_RECEIVER_H
#define ESP_NOW_RECEIVER_H

#include <esp_now.h>
#include <WiFi.h>

// 1. Define the Data Structure
// This MUST be identical on both the Transmitter and Receiver
typedef struct packetData {
  float num1;
  float num2;
  float num3;
} packetData;

// 2. Create Global Variables
// This variable will store the incoming data
packetData receivedData;

// This "flag" will be set to true when new data arrives
// "volatile" is important to ensure the main loop sees the change
volatile bool newDataAvailable = false;


// 3. The "Data Received" Callback
// This function runs automatically when a packet arrives
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Copy the incoming data into our global 'receivedData' structure
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  // Set the flag so the main loop knows there's new data
  newDataAvailable = true;
}

// 4. The Main Setup Function
// This is called once from your main .ino's setup()
void setup_esp_now_receiver() {
  // Set ESP32 to Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the "data received" callback
  esp_now_register_recv_cb(OnDataRecv);
}

#endif // ESP_NOW_RECEIVER_H