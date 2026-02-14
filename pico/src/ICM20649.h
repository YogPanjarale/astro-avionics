#pragma once
#include <Arduino.h>
#include <Wire.h>

class ICM20649 {
public:
  ICM20649(TwoWire& wire, uint8_t addr = 0x68);

  bool begin();
  void update();   // NEW

  float getAccelX();
  float getAccelY();
  float getAccelZ();

  float getGyroX();
  float getGyroY();
  float getGyroZ();

  float getTemperature();

private:
  TwoWire* _wire;
  uint8_t  _addr;

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float temp = 0;
};
