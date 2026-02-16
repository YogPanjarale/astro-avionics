#pragma once
#include <Arduino.h>
#include <Wire.h>

class LIS3MDL {
public:
  LIS3MDL(TwoWire& wire, uint8_t addr = 0x1E);

  bool begin();
  void read(); 

  float getMagX();
  float getMagY();
  float getMagZ();

  float getTemperature();

private:
  TwoWire* _wire;
  uint8_t  _addr;

  float mx = 0, my = 0, mz = 0;
  float temp = 0;
};
