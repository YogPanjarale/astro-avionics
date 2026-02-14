#pragma once
#include <Arduino.h>
#include <Wire.h>

class LIS3MDL {
public:
  // Constructor with I2C bus
  LIS3MDL(TwoWire& wire, uint8_t addr = 0x1E);

  bool begin();

  // Magnetometer (uT)
  float getMagX();
  float getMagY();
  float getMagZ();

  // Temperature (°C)
  float getTemperature();

private:
  TwoWire* _wire;
  uint8_t  _addr;
};
