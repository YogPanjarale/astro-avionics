#include "ICM20649.h"

// Bank select
#define REG_BANK_SEL 0x7F

// Bank 0 registers
#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define ACCEL_XOUT_H 0x2D

// Bank 2 registers
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02
#define ACCEL_CONFIG 0x14
#define ACCEL_CONFIG_2 0x15

#define ICM20649_ID 0xE1

ICM20649::ICM20649(TwoWire &wire, uint8_t addr)
{
    _wire = &wire;
    _addr = addr;

    ax = ay = az = 0;
    gx = gy = gz = 0;
    temp = 0;
}

bool ICM20649::writeReg(uint8_t reg, uint8_t val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

bool ICM20649::readBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);

    if (_wire->endTransmission(false) != 0)
        return false;

    if (_wire->requestFrom(_addr, len) != len)
        return false;

    for (int i = 0; i < len; i++)
        buf[i] = _wire->read();

    return true;
}

bool ICM20649::begin()
{
    uint8_t id;

    if (!readBytes(WHO_AM_I, &id, 1))
        return false;

    if (id != ICM20649_ID)
        return false;

    // Reset
    writeReg(PWR_MGMT_1, 0x80);
    delay(100);

    // Wake up
    writeReg(PWR_MGMT_1, 0x01);
    delay(10);

    writeReg(PWR_MGMT_2, 0x00);
    delay(10);

    // Switch to bank 2
    writeReg(REG_BANK_SEL, 0x20);

    writeReg(ACCEL_CONFIG, 0x08);   // ±4g
    writeReg(GYRO_CONFIG_1, 0x18);  // ±2000 dps

    writeReg(ACCEL_CONFIG_2, 0x03);
    writeReg(GYRO_CONFIG_2, 0x03);

    // Return to bank 0
    writeReg(REG_BANK_SEL, 0x00);

    return true;
}

bool ICM20649::read()
{
    uint8_t buf[14];

    if (!readBytes(ACCEL_XOUT_H, buf, 14))
        return false;

    int16_t rawAx = (buf[0] << 8) | buf[1];
    int16_t rawAy = (buf[2] << 8) | buf[3];
    int16_t rawAz = (buf[4] << 8) | buf[5];

    int16_t rawTemp = (buf[6] << 8) | buf[7];

    int16_t rawGx = (buf[8] << 8) | buf[9];
    int16_t rawGy = (buf[10] << 8) | buf[11];
    int16_t rawGz = (buf[12] << 8) | buf[13];

    const float accelScale = 4.0f / 32768.0f;
    const float gyroScale  = 2000.0f / 32768.0f;

    ax = rawAx * accelScale * 9.80665f;
    ay = rawAy * accelScale * 9.80665f;
    az = rawAz * accelScale * 9.80665f;

    gx = rawGx * gyroScale;
    gy = rawGy * gyroScale;
    gz = rawGz * gyroScale;

    temp = (rawTemp / 333.87f) + 21.0f;

    return true;
}

float ICM20649::accelX() { return ax; }
float ICM20649::accelY() { return ay; }
float ICM20649::accelZ() { return az; }

float ICM20649::gyroX() { return gx; }
float ICM20649::gyroY() { return gy; }
float ICM20649::gyroZ() { return gz; }

float ICM20649::temperature() { return temp; }