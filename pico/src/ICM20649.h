#pragma once
#include <Arduino.h>
#include <Wire.h>

class ICM20649 {
public:
  // Constructor with I2C bus
  ICM20649(TwoWire& wire, uint8_t addr = 0x68);

  bool begin();

  // Accelerometer (m/s^2)
  float getAccelX();
  float getAccelY();
  float getAccelZ();

  // Gyroscope (deg/s)
  float getGyroX();
  float getGyroY();
  float getGyroZ();

  // Temperature (°C)
  float getTemperature();

private:
  TwoWire* _wire;
  uint8_t  _addr;
};
