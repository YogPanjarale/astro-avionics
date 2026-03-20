#pragma once
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

struct Vec3{
    float x;
    float y;
    float z;
};

class IMU{
    public:
    bool begin(TwoWire* bus);
    void calibrate();
    void getAccelGyroTemp(Vec3 *accel, Vec3 *gyro, float* t);
    void getAccelGyro(Vec3 *a, Vec3 *g);

    
    private:
    Adafruit_ICM20649 icm;
    Vec3 gyro_bias = {0.0f, 0.0f, 0.0f};
    Vec3 accel_bias = {0.0f, 0.0f, 0.0f};
    Vec3 last_gyro = {0.0f, 0.0f, 0.0f};
    Vec3 last_accel = {0.0f, 0.0f, 0.0f};
    unsigned long lasttime = 0;

    sensors_event_t accel_event;
    sensors_event_t gyro_event;
    sensors_event_t temp_event;
};

