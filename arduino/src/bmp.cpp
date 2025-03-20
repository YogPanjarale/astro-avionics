#include "bmp.h"
#include "bmp3.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#define seaLevelPressure 101325 // Standard sea-level pressure in Pascals

BMPSensor::BMPSensor(int sda, int scl) : sdaPin(sda), sclPin(scl) {}

bool BMPSensor::begin()
{
    // start serial port at 115200 bps:
    Serial.begin(9600);
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);
    Wire.begin(sdaPin, sclPin); // Start I2C with defined pins

    // setup bmp sensor
    if (!bmp.begin_I2C(119))
    { // Initialize BMP390 over I2C
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        return false;
    }

    // oversampling to reduce noise and improve accuracy
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X); // new value should be updated every 40ms (25Hz)
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);  // new value should be updated every 40ms (25Hz)

    // // filtering to reduce noise
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    return true;
}

float BMPSensor::getAltitude()
{
    if (!bmp.performReading())
    {
        Serial.println("Faild to read from BMP390 sensor!");
        return -1; // Return -1 in case of failure
    }

    float pressure = bmp.pressure;       // Pressure in Pascals
    float temperature = bmp.temperature; // Temperature in Celsius

    // Convert pressure to altitude using the barometric formula
    // using the standard barometric formula h = (T0/L)(1-(P/P0)^(RL/gM))
    //  T0 is sea level temp, L is temp. lapse rate, P0 is sea level pressure, R is gas const,
    //  M is molar mass of air
    float altitude = 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));

    return altitude;
}

float BMPSensor::getVelocity()
{   
    static unsigned long prevTime = 0;                   // Previous time
    static float prevVelocity = 0;                       // Previous velocity

    if (millis() - prevTime < 45) // Update velocity every 45s
    {
        return prevVelocity;
    }
    unsigned long currentTime = millis();                // Get current time
    float currentAltitude = getAltitude();               // Get current altitude
    float deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds
    float velocity = 0;                                  // Default velocity

    if (deltaTime > 0) // Avoid division by zero
    {
        velocity = (currentAltitude - prevAltitude) / deltaTime;
    }

    // Update previous values
    prevAltitude = currentAltitude;
    prevTime = currentTime;
    prevVelocity = velocity;
    return velocity;
}

float BMPSensor::getPressure()
{
    if (!bmp.performReading())
    {
        Serial.println("Faild to read from BMP390 sensor!");
        return -1; // Return -1 in case of failure
    }

    float pressure = bmp.pressure; // Pressure in Pascals
    return pressure;
}