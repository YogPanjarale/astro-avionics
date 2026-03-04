#include "LIS3MDL.h"

// Registers
#define LIS3MDL_WHO_AM_I   0x0F
#define LIS3MDL_CTRL_REG1  0x20
#define LIS3MDL_CTRL_REG2  0x21
#define LIS3MDL_CTRL_REG3  0x22
#define LIS3MDL_CTRL_REG4  0x23
#define LIS3MDL_OUT_X_L    0x28
#define LIS3MDL_TEMP_OUT_L 0x2E

#define LIS3MDL_ID         0x3D

LIS3MDL::LIS3MDL(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

bool LIS3MDL::begin() {
  _wire->begin();

  // Check WHO_AM_I
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_WHO_AM_I);
  if (_wire->endTransmission(false) != 0) return false;

  _wire->requestFrom(_addr, (uint8_t)1);
  if (!_wire->available()) return false;

  uint8_t id = _wire->read();
  if (id != LIS3MDL_ID) return false;

  // CTRL_REG1
  // Temp enable | Ultra-high performance | 80Hz ODR
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_CTRL_REG1);
  _wire->write(0b11110000);
  _wire->endTransmission();

  // CTRL_REG2
  // ±4 gauss full scale
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_CTRL_REG2);
  _wire->write(0b00000000);
  _wire->endTransmission();

  // CTRL_REG3
  // Continuous conversion mode
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_CTRL_REG3);
  _wire->write(0b00000000);
  _wire->endTransmission();

  // CTRL_REG4
  // Ultra-high performance for Z
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_CTRL_REG4);
  _wire->write(0b00001100);
  _wire->endTransmission();

  return true;
}

void LIS3MDL::read() {
  uint8_t buffer[6];

  // Read 6 bytes starting from OUT_X_L with auto-increment
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_OUT_X_L | 0x80);  // auto-increment
  _wire->endTransmission(false);

  _wire->requestFrom(_addr, (uint8_t)6);

  if (_wire->available() == 6) {
    for (int i = 0; i < 6; i++) {
      buffer[i] = _wire->read();
    }

    int16_t x = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t y = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t z = (int16_t)(buffer[5] << 8 | buffer[4]);

    // ±4 gauss scale = 6842 LSB/gauss
    const float sensitivity = 6842.0;

    mx = x / sensitivity;
    my = y / sensitivity;
    mz = z / sensitivity;
  }

  // ---- Temperature ----
  _wire->beginTransmission(_addr);
  _wire->write(LIS3MDL_TEMP_OUT_L | 0x80);
  _wire->endTransmission(false);

  _wire->requestFrom(_addr, (uint8_t)2);

  if (_wire->available() == 2) {
    int16_t t = (int16_t)(_wire->read() | (_wire->read() << 8));
    temp = 25.0 + (t / 8.0);   // per datasheet
  }
}

float LIS3MDL::getMagX() { return mx; }
float LIS3MDL::getMagY() { return my; }
float LIS3MDL::getMagZ() { return mz; }

float LIS3MDL::getTemperature() { return temp; }