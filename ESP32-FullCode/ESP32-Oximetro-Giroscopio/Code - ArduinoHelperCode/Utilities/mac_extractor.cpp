// Utility file: mac_extractor.cpp

#include "WiFi.h"
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("My MAC Address is: ");
  Serial.println(macAddress);
}

void loop() {
  Serial.println("My MAC Address is: ");
  Serial.println(macAddress);
  delay(1000);
}