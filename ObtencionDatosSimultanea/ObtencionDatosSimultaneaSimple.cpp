// Example: BasicRead for ESP32-WROOM + MPU6500
#include <Wire.h>
#include <MPU6500.h>
#include "MAX30105.h"
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x68 // change to 0x69 if AD0 is HIGH

MPU6500 mpu;
MAX30105 ppg;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[BasicRead] Start");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(200);

  ppg.setup(); // configuración por defecto
  ppg.setPulseAmplitudeRed(0x2F);   // LED rojo moderado
  ppg.setPulseAmplitudeIR(0x2F);    // LED IR moderado

  if (!mpu.begin(Wire, I2C_ADDR)) {
    Serial.println("[BasicRead] MPU6500 not detected (WHO_AM_I != 0x70). Check wiring/address.");
    while (1) { delay(1000); }
  }

  // Optional: calibrate while sensor is still
  mpu.calibrate(400);
  Serial.println("READY");
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  mpu.readAccelG(ax, ay, az);
  mpu.readGyroDps(gx, gy, gz);

  long ir = ppg.getIR();
  long red = ppg.getRed();
  Serial.print("IR:"); Serial.print(ir);
  Serial.print(",RED:"); Serial.print(red);

  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);

  delay(500);
}
