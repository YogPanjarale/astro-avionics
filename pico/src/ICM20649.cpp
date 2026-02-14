#include "ICM20649.h"

// ----------------------

ICM20649::ICM20649(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

// ----------------------

bool ICM20649::begin() {
  // Later:
  // use _wire to check WHO_AM_I
  // configure ranges

  return true;
}

// ----------------------
// Accelerometer

float ICM20649::getAccelX() {
  return 0.0;
}

float ICM20649::getAccelY() {
  return 0.0;
}

float ICM20649::getAccelZ() {
  return 0.0;
}

// ----------------------
// Gyroscope

float ICM20649::getGyroX() {
  return 0.0;
}

float ICM20649::getGyroY() {
  return 0.0;
}

float ICM20649::getGyroZ() {
  return 0.0;
}

// ----------------------
// Temperature

float ICM20649::getTemperature() {
  return 0.0;
}
