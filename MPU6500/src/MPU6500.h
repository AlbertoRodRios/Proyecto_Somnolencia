#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal MPU6500 library (ESP32-friendly) by ChatGPT
// WHO_AM_I expected: 0x70

class MPU6500 {
public:
  enum AccRange : uint8_t { ACCEL_2G=0, ACCEL_4G=1, ACCEL_8G=2, ACCEL_16G=3 };
  enum GyrRange : uint8_t { GYRO_250DPS=0, GYRO_500DPS=1, GYRO_1000DPS=2, GYRO_2000DPS=3 };

  MPU6500() : _wire(&Wire), _addr(0x68), _accelSens(8192.0f), _gyroSens(65.5f),
                    _axOff(0), _ayOff(0), _azOff(0), _gxOff(0), _gyOff(0), _gzOff(0) {}

  bool begin(TwoWire& wire = Wire, uint8_t address = 0x68);
  bool isConnected();
  uint8_t whoAmI();

  void reset();
  void sleep(bool enable);

  void setSampleRateDivider(uint8_t div);     // SMPLRT_DIV
  void setDlpf(uint8_t dlpf);                 // CONFIG + ACCEL_CONFIG2 (0..7)

  void setAccelRange(AccRange r);
  void setGyroRange(GyrRange r);

  // Raw reads
  void readRawAccel(int16_t& ax, int16_t& ay, int16_t& az);
  void readRawGyro (int16_t& gx, int16_t& gy, int16_t& gz);
  void readRaw     (int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz);

  // Scaled reads
  void readAccelG(float& ax_g, float& ay_g, float& az_g);
  void readGyroDps(float& gx_dps, float& gy_dps, float& gz_dps);

  // Simple offset calibration (keep sensor still)
  void calibrate(uint16_t samples = 500);

  // Manual offsets
  void setAccelOffsets(int16_t ax, int16_t ay, int16_t az) { _axOff=ax; _ayOff=ay; _azOff=az; }
  void setGyroOffsets (int16_t gx, int16_t gy, int16_t gz) { _gxOff=gx; _gyOff=gy; _gzOff=gz; }

  uint8_t address() const { return _addr; }

private:
  // Registers
  static constexpr uint8_t REG_WHO_AM_I     = 0x75; // expect 0x70
  static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
  static constexpr uint8_t REG_CONFIG       = 0x1A; // GYRO DLPF
  static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
  static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t REG_ACCEL_CONFIG2= 0x1D; // ACCEL DLPF
  static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B; // then 14 bytes (accX..gyroZ)

  TwoWire* _wire;
  uint8_t  _addr;
  float    _accelSens;   // LSB per g -> use to convert: g = raw / _accelSens
  float    _gyroSens;    // LSB per dps -> dps = raw / _gyroSens

  // Offsets (raw units)
  int16_t _axOff, _ayOff, _azOff;
  int16_t _gxOff, _gyOff, _gzOff;

  void writeByte(uint8_t reg, uint8_t val);
  uint8_t readByte(uint8_t reg);
  void readBytes(uint8_t reg, uint8_t* buf, uint8_t len);
};
