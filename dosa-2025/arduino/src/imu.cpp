#include "imu.h"

Adafruit_ICM20649 icm;

// Calibration variables
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float velocityY = 0;
float height = 0;
unsigned long prevTime = 0;

// return 1 if setup is successful, 0 otherwise
int setupIMU(){
    if (!icm.begin_I2C()) {
        Serial.println("Failed to find ICM20649 chip");
        return 0;
    }

    Serial.println("ICM20649 initialized.");
    icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
    return 1;
}

void calibrateIMU(int samples)
{
    Serial.println("Calibrating... Keep the IMU steady!");

    float axSum = 0, aySum = 0, azSum = 0;
    float gxSum = 0, gySum = 0, gzSum = 0;

    for (int i = 0; i < samples; i++)
    {
        sensors_event_t accel, gyro, temp;
        icm.getEvent(&accel, &gyro, &temp);

        axSum += accel.acceleration.x;
        aySum += accel.acceleration.y;
        azSum += accel.acceleration.z; // we don't do minus for gravity

        gxSum += gyro.gyro.x;
        gySum += gyro.gyro.y;
        gzSum += gyro.gyro.z;

        delay(5); // Small delay between readings
    }

    accelBiasX = axSum / samples;
    accelBiasY = aySum / samples;
    accelBiasZ = azSum / samples;

    gyroBiasX = gxSum / samples;
    gyroBiasY = gySum / samples;
    gyroBiasZ = gzSum / samples;

    Serial.println("IMU Calibration Complete!");
}

IMUReading readIMU()
{
    sensors_event_t accel, gyro, temp;
    icm.getEvent(&accel, &gyro, &temp);

    IMUReading reading;
    reading.accel_x = accel.acceleration.x - accelBiasX;
    reading.accel_y = accel.acceleration.y - accelBiasY;
    reading.accel_z = accel.acceleration.z - accelBiasZ;
    reading.roll = atan2(accel.acceleration.y - accelBiasY, accel.acceleration.z - accelBiasZ) * 180 / PI;
    reading.pitch = atan2(-(accel.acceleration.x - accelBiasX),
                          sqrt(pow(accel.acceleration.y - accelBiasY, 2) + pow(accel.acceleration.z - accelBiasZ, 2))) *
                    180 / PI;

    reading.yaw = (gyro.gyro.z - gyroBiasZ); // Only raw value, fusion needed for accurate yaw

    return reading;
}

float getVelocityIMU()
{
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    sensors_event_t accel, gyro, temp;
    icm.getEvent(&accel, &gyro, &temp);

    float accelY = accel.acceleration.y - accelBiasY; // Remove bias
    
    //REMOVE NOISE???

    velocityY += accelY * dt; // Integrate acceleration to get velocity
    return velocityY;
}

float getHeightIMU(){
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    sensors_event_t accel, gyro, temp;
    icm.getEvent(&accel, &gyro, &temp);

    float accelY = accel.acceleration.y - accelBiasY; // Remove bias
    
    //REMOVE NOISE???

    velocityY += accelY * dt; // Integrate acceleration to get velocity
    height += velocityY * dt; // Integrate velocity to get height
    return height;
}

bool isRocketTippingOver()
{
    sensors_event_t accel, gyro, temp;
    icm.getEvent(&accel, &gyro, &temp);

    float pitch = atan2(-(accel.acceleration.x - accelBiasX),
                        sqrt(pow(accel.acceleration.y - accelBiasY, 2) + pow(accel.acceleration.z - accelBiasZ, 2))) * 180 / PI;

    float gyroX = gyro.gyro.x - gyroBiasX;
    float accelY = accel.acceleration.y - accelBiasY;

    // Define thresholds
    const float PITCH_THRESHOLD = 45.0;  // Degrees
    const float GYRO_X_THRESHOLD = 1.0;  // Rad/s (adjust based on testing)
    const float ACCEL_Y_DROP_THRESHOLD = 3.0;  // m/s² (low acceleration means freefall)

    // Check if pitch is too large or tipping too fast
    if (abs(pitch) > PITCH_THRESHOLD && abs(gyroX) > GYRO_X_THRESHOLD && accelY < ACCEL_Y_DROP_THRESHOLD)
    {
        return true;  // Rocket is tipping over
    }
    return false;
}
