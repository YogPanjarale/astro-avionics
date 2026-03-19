#include "imu_lib.h"

#define SAMPLES 2000
#define DELAY 5 // 200hz polling

bool IMU::begin(TwoWire *bus){
    if (!icm.begin_I2C(0x68, bus))
    {
        return false; // Failed to connect case
    }

    icm.setAccelRange(ICM20649_ACCEL_RANGE_16_G);   // Allowed: 4, 8, 16, 30
    icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS); // Allowed: 500, 1000, 2000, 4000
    icm.setAccelRateDivisor(4);                     //  @225Hz accel_rate = 1125 / (1 + accel_divisor)
    icm.setGyroRateDivisor(4);                      //  @220Hz gyro_rate = 1100 / (1 + gyro_divisor)

    return true;
    /*
    Might want to set a higher I2C bus speed
    We might poll the icm for avg/lowpass filtering
    The chip has DLPF onboard but couldnt find documentation revealing library functions that set it
    To set I2C speed at 400KHz in main:
    Wire.setClock(400000);
    */
}

void IMU::calibrate(){
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;

    delay(500); // let system settle

    for (uint16_t i = 0; i < SAMPLES; i++)
    {

        icm.getEvent(&accel_event, &gyro_event, NULL);

        sum_gx += gyro_event.gyro.x;
        sum_gy += gyro_event.gyro.y;
        sum_gz += gyro_event.gyro.z;

        sum_ax += accel_event.acceleration.x;
        sum_ay += accel_event.acceleration.y;
        sum_az += accel_event.acceleration.z;

        delay(DELAY); // running @200Hz
    }

    // -------- Gyro bias --------
    gyro_bias.x = sum_gx / SAMPLES;
    gyro_bias.y = sum_gy / SAMPLES;
    gyro_bias.z = sum_gz / SAMPLES;

    // -------- Accelerometer bias --------
    float avg_ax = sum_ax / SAMPLES;
    float avg_ay = sum_ay / SAMPLES;
    float avg_az = sum_az / SAMPLES;

    // Rocket upright assumption:
    // X ≈ 0
    // Y ≈ 0
    // Z ≈ +9.81 m/s^2

    accel_bias.x = avg_ax;
    accel_bias.y = avg_ay;
    accel_bias.z = avg_az - 9.81f;
}

void IMU::getAccelGyroTemp(Vec3 *a, Vec3 *g, float* t){
    icm.getEvent(&accel_event, &gyro_event, &temp_event);
    
    a->x = accel_event.acceleration.x - accel_bias.x;
    a->y = accel_event.acceleration.y - accel_bias.y;
    a->z = accel_event.acceleration.z - accel_bias.z;

    g->x = gyro_event.gyro.x - gyro_bias.x;
    g->y = gyro_event.gyro.y - gyro_bias.y;
    g->z = gyro_event.gyro.z - gyro_bias.z;

    *t = temp_event.temperature;
}

void IMU::getAccelGyro(Vec3 *a, Vec3 *g){  // Store these!!
    icm.getEvent(&accel_event, &gyro_event, NULL);
    
    a->x = accel_event.acceleration.x - accel_bias.x;
    a->y = accel_event.acceleration.y - accel_bias.y;
    a->z = accel_event.acceleration.z - accel_bias.z;

    g->x = gyro_event.gyro.x - gyro_bias.x;
    g->y = gyro_event.gyro.y - gyro_bias.y;
    g->z = gyro_event.gyro.z - gyro_bias.z;
}
// need to get gyro orientation to make local coordinates into world
// see arduino std madgwick quarternion library