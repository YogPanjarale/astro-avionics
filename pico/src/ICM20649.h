#pragma once
#include <Arduino.h>
#include <Wire.h>

class ICM20649 {

public:
    ICM20649(TwoWire &wire, uint8_t addr = 0x68);

    bool begin();
    bool read();

    float accelX();
    float accelY();
    float accelZ();

    float gyroX();
    float gyroY();
    float gyroZ();

    float temperature();

private:
    TwoWire *_wire;
    uint8_t _addr;

    bool writeReg(uint8_t reg, uint8_t val);
    bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len);

    float ax, ay, az;
    float gx, gy, gz;
    float temp;
};