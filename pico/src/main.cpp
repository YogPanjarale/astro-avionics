#define __FREERTOS 1

#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "LIS3MDL.h"
#include "ICM20649.h"

#define SDA_PIN 10
#define SCL_PIN 11

#define MAG_ADDR 0x1E
#define ICM_ADDR 0x68

// Sensors on SAME bus (Wire1)
LIS3MDL lis(Wire1, MAG_ADDR);
ICM20649 icm(Wire1, ICM_ADDR);

// Mutex for I2C
SemaphoreHandle_t i2cMutex;

// --------------------
// Task: Magnetometer
// --------------------
void magTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    lis.read();
    xSemaphoreGive(i2cMutex);

    Serial.print("[MAG] ");
    Serial.print(lis.getMagX(), 3);
    Serial.print(", ");
    Serial.print(lis.getMagY(), 3);
    Serial.print(", ");
    Serial.print(lis.getMagZ(), 3);

    Serial.print(" | T: ");
    Serial.println(lis.getTemperature(), 2);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// --------------------
// Task: IMU
// --------------------
void icmTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    icm.read();
    xSemaphoreGive(i2cMutex);

    Serial.print("[ICM] Accel: ");
    Serial.print(icm.accelX(), 2);
    Serial.print(", ");
    Serial.print(icm.accelY(), 2);
    Serial.print(", ");
    Serial.print(icm.accelZ(), 2);

    Serial.print(" | Gyro: ");
    Serial.print(icm.gyroX(), 2);
    Serial.print(", ");
    Serial.print(icm.gyroY(), 2);
    Serial.print(", ");
    Serial.print(icm.gyroZ(), 2);

    Serial.print(" | Temp: ");
    Serial.println(icm.temperature(), 2);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// --------------------
// I2C Scanner
// --------------------
void scanI2C() {
  Serial.println("Scanning I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println();
}

// --------------------
// Setup
// --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== MAG + ICM TEST ===");

  // ✅ Correct bus
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();

  // Mutex
  i2cMutex = xSemaphoreCreateMutex();

  // Scan
  scanI2C();

  // Init MAG
  Serial.println("Init LIS...");
  if (!lis.begin()) {
    Serial.println("❌ LIS failed");
    while (1);
  }
  Serial.println("✅ LIS OK");

  // Init ICM
  Serial.println("Init ICM...");
  if (!icm.begin()) {
    Serial.println("❌ ICM failed");
    while (1);
  }
  Serial.println("✅ ICM OK\n");

  // Tasks
  xTaskCreate(magTask, "MAG", 2048, NULL, 1, NULL);
  xTaskCreate(icmTask, "ICM", 2048, NULL, 1, NULL);
}

void loop() {}