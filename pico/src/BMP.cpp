#include "BMP.h"
#include <math.h>

// ----------------------

BMP::BMP(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

// ----------------------

bool BMP::begin() {
  // Later: use _wire to talk to sensor
  lastTime = millis();
  return true;
}

// ----------------------

void BMP::setSeaLevelPressure(float p) {
  seaLevelPressure = p;
}

// ----------------------

float BMP::getTemperature() {
  return 0.0;   // placeholder
}

// ----------------------

float BMP::getPressure() {
  return 1013.25;  // placeholder
}

// ----------------------

float BMP::getAltitude() {
  float p = getPressure();
  return 44330.0 * (1.0 - pow(p / seaLevelPressure, 0.1903));
}

// ----------------------

float BMP::getVelocity() {
  float v = 0;
  return v;
}
