// This is your main .ino file (e.g., Receiver_Main.ino)

#include "esp_now_reciever.h" // Import all our ESP-NOW functions

// For the "waiting" message heartbeat
unsigned long lastDelayMsg = 0;
const long delayInterval = 5000; // Print "waiting" every 5 seconds

void setup() {
  Serial.begin(112500); // Corrected to 115200
  Serial.println("\n--- ESP-NOW RECEIVER ---");

  // Call the single setup function from our new header
  setup_esp_now_receiver();

  // --- ADD YOUR OTHER SETUP CODE HERE ---
  Serial.println("Custom setup complete. Waiting for data...");
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
    
    Serial.println("\n[MAIN LOOP] Processing new data packet:");
    Serial.println("---------------------------------------");
    Serial.print("  Float 1: ");
    Serial.println(receivedData.num1);
    Serial.print("  Float 2: ");
    Serial.println(receivedData.num2);
    Serial.print("  Float 3: ");
    Serial.println(receivedData.num3);
    Serial.println("---------------------------------------");

    //
    // --- DO SOMETHING WITH THE DATA HERE ---
    // (e.g., control motors, update a display)
    //
    
    lastDelayMsg = millis(); // Reset the "waiting" timer
    
  } else {
    // --- Print a "waiting" message every 5 seconds ---
    // This lets you know the loop is still running
    // if you haven't received data in a while.
    if (millis() - lastDelayMsg > delayInterval) {
      Serial.println("(Main loop is running, waiting for data...)");
      lastDelayMsg = millis();
    }
  }

  // Your main loop can do other things here without being blocked
  delay(10); 
}