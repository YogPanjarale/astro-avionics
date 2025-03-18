#include <Wire.h>
#include <Adafruit_ICM20649.h>

Adafruit_ICM20649 icm;

struct IMUReading {
    double roll;
    double yaw;
    double pitch;
    double acceleration;
};

// Calibration variables
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void calibrateIMU(int samples = 500) {
    Serial.println("Calibrating... Keep the IMU steady!");

    float axSum = 0, aySum = 0, azSum = 0;
    float gxSum = 0, gySum = 0, gzSum = 0;

    for (int i = 0; i < samples; i++) {
        sensors_event_t accel, gyro, temp;
        icm.getEvent(&accel, &gyro, &temp);

        axSum += accel.acceleration.x;
        aySum += accel.acceleration.y;
        azSum += accel.acceleration.z - 9.81; // Remove gravity

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

    Serial.println("Calibration Complete!");
}

IMUReading readIMU() {
    sensors_event_t accel, gyro, temp;
    icm.getEvent(&accel, &gyro, &temp);

    IMUReading reading;
    reading.acceleration = sqrt(pow(accel.acceleration.x - accelBiasX, 2) +
                                pow(accel.acceleration.y - accelBiasY, 2) +
                                pow(accel.acceleration.z - accelBiasZ, 2));

    reading.roll = atan2(accel.acceleration.y - accelBiasY, accel.acceleration.z - accelBiasZ) * 180 / PI;
    reading.pitch = atan2(-(accel.acceleration.x - accelBiasX), 
                          sqrt(pow(accel.acceleration.y - accelBiasY, 2) + pow(accel.acceleration.z - accelBiasZ, 2))) * 180 / PI;

    reading.yaw = (gyro.gyro.z - gyroBiasZ); // Only raw value, fusion needed for accurate yaw

    return reading;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!icm.begin_I2C()) {
        Serial.println("Failed to find ICM20649 chip");
        while (1);
    }

    Serial.println("ICM20649 initialized.");
    icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
    
    // Perform calibration
    calibrateIMU();
}

void loop() {
    IMUReading reading = readIMU();

    Serial.print("Acceleration: "); Serial.println(reading.acceleration);
    Serial.print("Roll: "); Serial.println(reading.roll);
    Serial.print("Yaw: "); Serial.println(reading.yaw);
    Serial.print("Pitch: "); Serial.println(reading.pitch);
    Serial.println("------------------------");

    delay(1000);
}