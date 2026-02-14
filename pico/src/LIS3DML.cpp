#include "LIS3MDL.h"

// ----------------------

LIS3MDL::LIS3MDL(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

// ----------------------

bool LIS3MDL::begin() {
  // Later:
  // use _wire to check WHO_AM_I
  // configure ranges

  return true;
}

// ----------------------
// Magnetometer

float LIS3MDL::getMagX() {
  return 0.0;
}

float LIS3MDL::getMagY() {
  return 0.0;
}

float LIS3MDL::getMagZ() {
  return 0.0;
}

// ----------------------
// Temperature

float LIS3MDL::getTemperature() {
  return 0.0;
}
