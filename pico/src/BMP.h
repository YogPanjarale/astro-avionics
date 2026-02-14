#pragma once
#include <Arduino.h>
#include <Wire.h>

class BMP {
public:
  BMP(TwoWire& wire, uint8_t addr = 0x77);

  bool begin();
  void update();   // NEW

  void setSeaLevelPressure(float p);

  float getTemperature();
  float getPressure();
  float getAltitude();
  float getVelocity();

private:
  TwoWire* _wire;
  uint8_t  _addr;

  float seaLevelPressure = 1013.25;

  float temperature = 0;
  float pressure = 1013.25;

  float lastAltitude = 0;
  unsigned long lastTime = 0;
};
