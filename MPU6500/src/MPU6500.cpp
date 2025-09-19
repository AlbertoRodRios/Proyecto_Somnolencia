#include "MPU6500.h"

bool MPU6500::begin(TwoWire& wire, uint8_t address) {
  _wire = &wire;
  _addr = address;

  // Do NOT call _wire->begin() here (let the sketch set pins if needed).
  delay(50);

  // Check WHO_AM_I (should be 0x70 for MPU6500)
  uint8_t who = whoAmI();
  if (who != 0x70) {
    // Try the other I2C address if WHO_AM_I failed at first address guess
    _addr = (address == 0x68) ? 0x69 : 0x68;
    who = whoAmI();
    if (who != 0x70) return false;
  }

  // Wake up device
  writeByte(REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Default config: DLPF ~ 44 Hz (gyro & accel), SRD=4 (~200 Hz from 1kHz)
  setDlpf(0x03);
  setSampleRateDivider(4);

  // Default ranges
  setAccelRange(ACCEL_4G);
  setGyroRange(GYRO_500DPS);

  return true;
}

bool MPU6500::isConnected() {
  return whoAmI() == 0x70;
}

uint8_t MPU6500::whoAmI() {
  return readByte(REG_WHO_AM_I);
}

void MPU6500::reset() {
  // set DEVICE_RESET bit (bit7) in PWR_MGMT_1
  writeByte(REG_PWR_MGMT_1, 0x80);
  delay(100);
  // wake up
  writeByte(REG_PWR_MGMT_1, 0x00);
  delay(100);
}

void MPU6500::sleep(bool enable) {
  uint8_t v = readByte(REG_PWR_MGMT_1);
  if (enable) v |= 0x40; else v &= ~0x40;
  writeByte(REG_PWR_MGMT_1, v);
}

void MPU6500::setSampleRateDivider(uint8_t div) {
  writeByte(REG_SMPLRT_DIV, div);
}

void MPU6500::setDlpf(uint8_t dlpf) {
  dlpf &= 0x07; // 0..7
  // Gyro DLPF in CONFIG (bits 2:0)
  uint8_t c = readByte(REG_CONFIG);
  c = (c & ~0x07) | dlpf;
  writeByte(REG_CONFIG, c);
  // Accel DLPF in ACCEL_CONFIG2 (bits 2:0)
  uint8_t a2 = readByte(REG_ACCEL_CONFIG2);
  a2 = (a2 & ~0x07) | dlpf;
  writeByte(REG_ACCEL_CONFIG2, a2);
}

void MPU6500::setAccelRange(AccRange r) {
  uint8_t v = readByte(REG_ACCEL_CONFIG);
  v &= ~(0x03 << 3);      // clear AFS bits
  v |= ((uint8_t)r << 3); // set range
  writeByte(REG_ACCEL_CONFIG, v);

  switch (r) {
    case ACCEL_2G:  _accelSens = 16384.0f; break;
    case ACCEL_4G:  _accelSens = 8192.0f;  break;
    case ACCEL_8G:  _accelSens = 4096.0f;  break;
    case ACCEL_16G: _accelSens = 2048.0f;  break;
  }
}

void MPU6500::setGyroRange(GyrRange r) {
  uint8_t v = readByte(REG_GYRO_CONFIG);
  v &= ~(0x03 << 3);      // clear FS bits
  v |= ((uint8_t)r << 3); // set range
  writeByte(REG_GYRO_CONFIG, v);

  switch (r) {
    case GYRO_250DPS:  _gyroSens = 131.0f;  break;
    case GYRO_500DPS:  _gyroSens = 65.5f;   break;
    case GYRO_1000DPS: _gyroSens = 32.8f;   break;
    case GYRO_2000DPS: _gyroSens = 16.4f;   break;
  }
}

void MPU6500::readRawAccel(int16_t& ax, int16_t& ay, int16_t& az) {
  uint8_t buf[6];
  readBytes(REG_ACCEL_XOUT_H, buf, 6);
  ax = (int16_t)((buf[0] << 8) | buf[1]) - _axOff;
  ay = (int16_t)((buf[2] << 8) | buf[3]) - _ayOff;
  az = (int16_t)((buf[4] << 8) | buf[5]) - _azOff;
}

void MPU6500::readRawGyro(int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[6];
  readBytes(REG_ACCEL_XOUT_H + 8, buf, 6); // gyro starts at 0x43
  gx = (int16_t)((buf[0] << 8) | buf[1]) - _gxOff;
  gy = (int16_t)((buf[2] << 8) | buf[3]) - _gyOff;
  gz = (int16_t)((buf[4] << 8) | buf[5]) - _gzOff;
}

void MPU6500::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                            int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  readBytes(REG_ACCEL_XOUT_H, buf, 14);
  ax = (int16_t)((buf[0] << 8) | buf[1]) - _axOff;
  ay = (int16_t)((buf[2] << 8) | buf[3]) - _ayOff;
  az = (int16_t)((buf[4] << 8) | buf[5]) - _azOff;
  gx = (int16_t)((buf[8] << 8) | buf[9]) - _gxOff;
  gy = (int16_t)((buf[10] << 8) | buf[11]) - _gyOff;
  gz = (int16_t)((buf[12] << 8) | buf[13]) - _gzOff;
}

void MPU6500::readAccelG(float& ax_g, float& ay_g, float& az_g) {
  int16_t ax, ay, az;
  readRawAccel(ax, ay, az);
  ax_g = (float)ax / _accelSens;
  ay_g = (float)ay / _accelSens;
  az_g = (float)az / _accelSens;
}

void MPU6500::readGyroDps(float& gx_dps, float& gy_dps, float& gz_dps) {
  int16_t gx, gy, gz;
  readRawGyro(gx, gy, gz);
  gx_dps = (float)gx / _gyroSens;
  gy_dps = (float)gy / _gyroSens;
  gz_dps = (float)gz / _gyroSens;
}

void MPU6500::calibrate(uint16_t samples) {
  long ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  for (uint16_t i=0; i<samples; i++) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    readRaw(rax, ray, raz, rgx, rgy, rgz);
    ax += rax; ay += ray; az += raz;
    gx += rgx; gy += rgy; gz += rgz;
    delay(3);
  }
  _axOff = (int16_t)(ax / (long)samples);
  _ayOff = (int16_t)(ay / (long)samples);
  // Para el eje Z, resta la gravedad aproximada en raw (depende de rango).
  // Si usas ±4g (8192 LSB/g), 1g ≈ 8192 LSB
  _azOff = (int16_t)(az / (long)samples - (int16_t)_accelSens);
  _gxOff = (int16_t)(gx / (long)samples);
  _gyOff = (int16_t)(gy / (long)samples);
  _gzOff = (int16_t)(gz / (long)samples);
}

// ---------------- I2C helpers ----------------
void MPU6500::writeByte(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t MPU6500::readByte(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_addr, (uint8_t)1);
  if (_wire->available()) return _wire->read();
  return 0xFF;
}

void MPU6500::readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_addr, len);
  for (uint8_t i=0; i<len && _wire->available(); i++) {
    buf[i] = _wire->read();
  }
}
