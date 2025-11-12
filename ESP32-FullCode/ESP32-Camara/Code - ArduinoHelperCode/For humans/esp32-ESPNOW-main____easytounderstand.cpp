// This is your main .ino file (e.g., Receiver_Main.ino)

#include "esp_now_reciever.h" // Import all our ESP-NOW functions

void setup() {
  Serial.begin(115200);

  // Call the single setup function from our new header
  setup_esp_now_receiver();

  // --- ADD YOUR OTHER SETUP CODE HERE ---
  // (e.g., display init, motor setup, etc.)
  //
}

void loop() {
  // --- YOUR MAIN LOGIC HERE ---
  // (This code runs constantly)
  //
  
  // Check if the flag has been set by the ESP-NOW callback
  if (newDataAvailable) {
    
    // Reset the flag
    newDataAvailable = false; 

    // Now you can safely use the data from the 'receivedData' struct
    // (Both the flag and the struct are defined in the .h file)
    
    Serial.println("--- New Data Processed in Main Loop ---");
    Serial.print("Float 1: ");
    Serial.println(receivedData.num1);
    Serial.print("Float 2: ");
    Serial.println(receivedData.num2);
    Serial.print("Float 3: ");
    Serial.println(receivedData.num3);
    Serial.println("---------------------------------------");

    //
    // --- DO SOMETHING WITH THE DATA HERE ---
    // (e.g., control motors, update a display)
    //
  }

  // Your main loop can do other things here without being blocked
  delay(10); 
}