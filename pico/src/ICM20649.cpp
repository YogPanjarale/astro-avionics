#include "ICM20649.h"

ICM20649::ICM20649(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

bool ICM20649::begin() {
  return true;
}

void ICM20649::read() {
  // Later: read sensor registers
  ax = ay = 0.0;
  az = 9.81;   // pretend gravity

  gx = gy = gz = 0.0;
  temp = 25.0;
}

float ICM20649::getAccelX() { return ax; }
float ICM20649::getAccelY() { return ay; }
float ICM20649::getAccelZ() { return az; }

float ICM20649::getGyroX() { return gx; }
float ICM20649::getGyroY() { return gy; }
float ICM20649::getGyroZ() { return gz; }

float ICM20649::getTemperature() { return temp; }
