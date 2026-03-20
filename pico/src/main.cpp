#define __FREERTOS 1

#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "LIS3MDL.h"
#include "ICM20649.h"
#include "BMP.h"

#define SDA_PIN 10
#define SCL_PIN 11

#define MAG_ADDR 0x1E
#define ICM_ADDR 0x68
#define BMP_ADDR 0x77

// Sensors
LIS3MDL lis(Wire1, MAG_ADDR);
ICM20649 icm(Wire1, ICM_ADDR);
BMP bmp(Wire1, BMP_ADDR);

// Mutexes
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;

// --------------------
// Shared Data Struct
// --------------------
typedef struct {
  uint32_t timestamp;

  // MAG
  float magX, magY, magZ, magTemp;

  // ICM
  float ax, ay, az;
  float gx, gy, gz;
  float imuTemp;

  // BMP
  float altitude;
  float velocity;
  float pressure;
  float bmpTemp;

} SensorData;

SensorData gData;

// --------------------
// Task: MAG
// --------------------
void magTask(void* param) {
  for (;;) {

    // Read sensor
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    lis.read();
    xSemaphoreGive(i2cMutex);

    // Update shared data
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.magX = lis.getMagX();
    gData.magY = lis.getMagY();
    gData.magZ = lis.getMagZ();
    gData.magTemp = lis.getTemperature();
    gData.timestamp = millis();

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz
  }
}

// --------------------
// Task: ICM
// --------------------
void icmTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    icm.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.ax = icm.accelX();
    gData.ay = icm.accelY();
    gData.az = icm.accelZ();

    gData.gx = icm.gyroX();
    gData.gy = icm.gyroY();
    gData.gz = icm.gyroZ();

    gData.imuTemp = icm.temperature();
    gData.timestamp = millis();

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
  }
}

// --------------------
// Task: BMP
// --------------------
void bmpTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bmp.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.altitude = bmp.getRelativeAltitude();
    gData.velocity = bmp.getVelocity();
    gData.pressure = bmp.getPressure();
    gData.bmpTemp = bmp.getTemperature();
    gData.timestamp = millis();

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
  }
}

// --------------------
// Task: PRINT
// --------------------
void printTask(void* param) {
  SensorData localCopy;

  for (;;) {

    // Copy data quickly (minimize lock time)
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    localCopy = gData;
    xSemaphoreGive(dataMutex);

    // Print OUTSIDE mutex
    Serial.print("[");
    Serial.print(localCopy.timestamp);
    Serial.println(" ms]");

    Serial.print("MAG: ");
    Serial.print(localCopy.magX, 3); Serial.print(", ");
    Serial.print(localCopy.magY, 3); Serial.print(", ");
    Serial.print(localCopy.magZ, 3);

    Serial.print(" | T: ");
    Serial.println(localCopy.magTemp, 2);

    Serial.print("ICM A: ");
    Serial.print(localCopy.ax, 2); Serial.print(", ");
    Serial.print(localCopy.ay, 2); Serial.print(", ");
    Serial.print(localCopy.az, 2);

    Serial.print(" | G: ");
    Serial.print(localCopy.gx, 2); Serial.print(", ");
    Serial.print(localCopy.gy, 2); Serial.print(", ");
    Serial.print(localCopy.gz, 2);

    Serial.print(" | T: ");
    Serial.println(localCopy.imuTemp, 2);

    Serial.print("BMP Alt: ");
    Serial.print(localCopy.altitude, 2);

    Serial.print(" m | Vel: ");
    Serial.print(localCopy.velocity, 2);

    Serial.print(" m/s | P: ");
    Serial.print(localCopy.pressure, 2);

    Serial.print(" hPa | T: ");
    Serial.println(localCopy.bmpTemp, 2);

    Serial.println("----------------------------------");
    Serial.println(millis());
    vTaskDelay(pdMS_TO_TICKS(200));  // slow print
  }
}

// --------------------
// Setup
// --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== SHARED DATA LOGGER ===");

  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);

  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  if (!lis.begin()) { Serial.println("❌ LIS fail"); while (1); }
  if (!icm.begin()) { Serial.println("❌ ICM fail"); while (1); }
  if (!bmp.begin()) { Serial.println("❌ BMP fail"); while (1); }

  Serial.println("✅ All sensors OK\n");

  xTaskCreate(icmTask, "ICM", 2048, NULL, 2, NULL);
  xTaskCreate(bmpTask, "BMP", 2048, NULL, 1, NULL);
  xTaskCreate(magTask, "MAG", 2048, NULL, 1, NULL);
  xTaskCreate(printTask, "PRINT", 4096, NULL, 0, NULL);
}

void loop() {}