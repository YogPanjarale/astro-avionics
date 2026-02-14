#include "BMP.h"
#include <math.h>

BMP::BMP(TwoWire& wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;
}

bool BMP::begin() {
  lastTime = millis();
  return true;
}

// Called periodically by task
void BMP::update() {
  // Later: read real sensor
  temperature = 25.0;     // dummy
  pressure = 1013.25;     // dummy
}

void BMP::setSeaLevelPressure(float p) {
  seaLevelPressure = p;
}

float BMP::getTemperature() {
  return temperature;
}

float BMP::getPressure() {
  return pressure;
}

float BMP::getAltitude() {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

float BMP::getVelocity() {
  float alt = getAltitude();
  unsigned long now = millis();

  float dt = (now - lastTime) / 1000.0;
  float v = 0;

  if (dt > 0) {
    v = (alt - lastAltitude) / dt;
  }

  lastAltitude = alt;
  lastTime = now;

  return v;
}
