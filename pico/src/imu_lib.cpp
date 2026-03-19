#include "imu_lib.h"

#define SAMPLES 2000
#define DELAY 5 // 200hz polling

bool IMU::begin(TwoWire *bus){
    if (!icm.begin_I2C(0x68, bus))
    {
        return false; // Failed to connect case
    }

    icm.setAccelRange(ICM20649_ACCEL_RANGE_16_G);   // Allowed: 4, 8, 16, 30
    icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS); // Allowed 500, 1000, 2000, 4000
    icm.setAccelRateDivisor(4);                     // accel_rate = 1125 / (1 + accel_divisor)
    icm.setGyroRateDivisor(4);                      // gyro_rate = 1100 / (1 + gyro_divisor)
    /*
    Might want to set a higher I2C bus speed
    We might poll the icm for avg/lowpass filtering
    The chip has DLPF onboard but couldnt find documentation revealing library functions that set it
    To set I2C speed at 400KHz in main:
    Wire.setClock(400000);
    */
}

void IMU::calibrate(){
    sensors_event_t accel;
    sensors_event_t gyro;

    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;

    delay(500); // let system settle

    for (uint16_t i = 0; i < SAMPLES; i++)
    {

        icm.getEvent(&accel, &gyro, NULL);

        sum_gx += gyro.gyro.x;
        sum_gy += gyro.gyro.y;
        sum_gz += gyro.gyro.z;

        sum_ax += accel.acceleration.x;
        sum_ay += accel.acceleration.y;
        sum_az += accel.acceleration.z;

        delay(DELAY); // running @200Hz
    }

    // -------- Gyro bias --------
    gyro_bias.x = sum_gx / samples;
    gyro_bias.y = sum_gy / samples;
    gyro_bias.z = sum_gz / samples;

    // -------- Accelerometer bias --------
    float avg_ax = sum_ax / samples;
    float avg_ay = sum_ay / samples;
    float avg_az = sum_az / samples;

    // Rocket upright assumption:
    // X ≈ 0
    // Y ≈ 0
    // Z ≈ +9.81 m/s^2

    accel_bias.x = avg_ax;
    accel_bias.y = avg_ay;
    accel_bias.z = avg_az - 9.81f;
}

void IMU::getAccelGyroTemp(Vec3 *a, Vec3 *g, float* t){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    icm.getEvent(&accel, &gyro, &temp);
    
    a->x = accel.acceleration.x - accel_bias.x;
    a->z = accel.acceleration.z - accel_bias.z;
    a->y = accel.acceleration.y - accel_bias.y;

    g->x = gyro.gyro.x - gyro_bias.x;
    g->y = gyro.gyro.y - gyro_bias.y;
    g->z = gyro.gyro.z - gyro_bias.z;

    *t = temp.temperature;
}

void IMU::getAccelGyro(Vec3 *a, Vec3 *g){  // Store these!!
    sensors_event_t accel;
    sensors_event_t gyro;

    icm.getEvent(&accel, &gyro, NULL);
    
    a->x = accel.acceleration.x - accel_bias.x;
    a->z = accel.acceleration.z - accel_bias.z;
    a->y = accel.acceleration.y - accel_bias.y;

    g->x = gyro.gyro.x - gyro_bias.x;
    g->y = gyro.gyro.y - gyro_bias.y;
    g->z = gyro.gyro.z - gyro_bias.z;
}
// need to get gyro orientation to make local coordinates into world
// see arduino std madgwick quarternion library