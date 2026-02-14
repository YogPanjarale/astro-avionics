#include "LIS3MDL.h"

LIS3MDL::LIS3MDL(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

bool LIS3MDL::begin() {
  return true;
}

void LIS3MDL::update() {
  // Later: read sensor
  mx = my = 0.0;
  mz = 50.0;   // fake earth field
  temp = 25.0;
}

float LIS3MDL::getMagX() { return mx; }
float LIS3MDL::getMagY() { return my; }
float LIS3MDL::getMagZ() { return mz; }

float LIS3MDL::getTemperature() { return temp; }
