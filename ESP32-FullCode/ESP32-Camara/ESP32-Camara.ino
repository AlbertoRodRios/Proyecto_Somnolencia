// This is your main .ino file (e.g., Sender_Main.ino)

#include "esp_now_sender.h" // Import all our ESP-NOW functions

// !! IMPORTANT !!
// REPLACE THIS with the MAC address of your Receiver board
//RECEIVER MAC ADDRESS: 04:83:08:75:93:EC
uint8_t receiverMACAddress[] = {0x04, 0x83, 0x08, 0x75, 0x93, 0xEC};

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP-NOW SENDER ---");

  // Call the single setup function from our new header
  setup_esp_now_sender(receiverMACAddress);

  // --- ADD YOUR OTHER SETUP CODE HERE ---
  // (e.g., camera init, pin setup, etc.)
  Serial.println("Custom setup complete. Starting main loop...");
}

void loop() {
  // --- YOUR MAIN LOGIC HERE ---
  // (e.g., get data from your camera or sensors)
  //

  Serial.println("\n---------------------------------");
  Serial.print("[MAIN LOOP] ");
  Serial.print(millis()); // Print timestamp
  Serial.println(": Creating new data packet...");

  // 1. Create a data structure and fill it
  packetData myData; // 'packetData' is defined in the .h file
  myData.num1 = 1.23;
  myData.num2 = random(100); // Use random data for testing
  myData.num3 = 9.87;
  
  // 2. Send the data
  // This function (now with feedback) is in the .h file
  send_data_packet(&myData);

  Serial.println("---------------------------------");

  delay(2000); // Your main loop delay
}