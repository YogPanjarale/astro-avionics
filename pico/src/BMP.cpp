#include "BMP.h"

BMP::BMP(TwoWire& wire, uint8_t addr) {
    _wire = &wire;
    _addr = addr;
}

bool BMP::begin() {
    // Initialize sensor on given I2C bus
    if (!bmp.begin_I2C(_addr, _wire)) {
        return false;
    }

    // Configure sensor (same tuning as earlier implementation)
    // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);  // 50Hz update rate

    // Take initial reading to establish baseline
    if (!bmp.performReading()) return false;

    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0f;

    altitude = bmp.readAltitude(seaLevelPressure);

    // Store ground reference (IMPORTANT for flight logic)
    groundAltitude = altitude;

    lastAltitude = altitude;
    lastTime = millis();

    return true;
}

void BMP::read() {
    unsigned long now = millis();
    unsigned long dt_ms = now - lastTime;

    // Limit update rate to sensor ODR (~50Hz)
    // Calling faster than this just adds noise
    if (dt_ms < 20) return;

    if (!bmp.performReading()) return;

    float dt = dt_ms / 1000.0f;

    // Update raw sensor values
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0f;

    // Compute absolute altitude (WARNING: depends on sea level pressure)
    altitude = bmp.readAltitude(seaLevelPressure);

    // Compute vertical velocity (m/s)
    float newVelocity = (altitude - lastAltitude) / dt;

    // Simple low-pass filter to reduce noise (important for flight use)
    velocity = 0.7f * velocity + 0.3f * newVelocity;

    lastAltitude = altitude;
    lastTime = now;
}

void BMP::setSeaLevelPressure(float p) {
    // Set this BEFORE begin() ideally
    // Incorrect value will shift altitude readings significantly
    seaLevelPressure = p;
}

float BMP::getTemperature() {
    return temperature;
}

float BMP::getPressure() {
    return pressure;
}

float BMP::getAltitude() {
    // WARNING: Absolute altitude (not relative to ground)
    return altitude;
}

float BMP::getRelativeAltitude() {
    // Use this for flight logic (liftoff, apogee, etc.)
    return altitude - groundAltitude;
}

float BMP::getVelocity() {
    // Filtered vertical velocity (m/s)
    return velocity;
}