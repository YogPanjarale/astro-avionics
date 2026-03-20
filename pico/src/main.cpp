#define __FREERTOS 1

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "LIS3MDL.h"
#include "ICM20649.h"
#include "BMP.h"

// --------------------
// PIN CONFIG
// --------------------
#define SDA_PIN 10
#define SCL_PIN 11

#define SPI_SCK  2
#define SPI_MOSI 3
#define SPI_MISO 4

#define SD_CS    26
#define LORA_CS  5

// --------------------
// SENSOR ADDR
// --------------------
#define MAG_ADDR 0x1E
#define ICM_ADDR 0x68
#define BMP_ADDR 0x77

// --------------------
// OBJECTS
// --------------------
LIS3MDL lis(Wire1, MAG_ADDR);
ICM20649 icm(Wire1, ICM_ADDR);
BMP bmp(Wire1, BMP_ADDR);

File logFile;

// --------------------
// MUTEX + QUEUE
// --------------------
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t spiMutex;

QueueHandle_t logQueue;

// --------------------
// DATA STRUCT
// --------------------
typedef struct {
  uint32_t time;

  float ax, ay, az;
  float gx, gy, gz;

  float mx, my, mz;

  float altitude;
  float velocity;
  float pressure;
  float temp_bmp;
  float temp_imu;
  uint32_t dt;

} SensorData;

SensorData gData;

// --------------------
// AUTO FILE NAME
// --------------------
String getNextFilename() {
  for (int i = 1; i < 1000; i++) {
    String name = "/log_" + String(i) + ".csv";
    if (!SD.exists(name)) return name;
  }
  return "/log_max.csv";
}

// --------------------
// IMU TASK
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

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
  }
}

// --------------------
// MAG TASK
// --------------------
void magTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    lis.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.mx = lis.getMagX();
    gData.my = lis.getMagY();
    gData.mz = lis.getMagZ();

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}

// --------------------
// BMP TASK
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

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
  }
}

// --------------------
// SAMPLER TASK
// --------------------
void samplerTask(void* param) {
  SensorData d;

  for (;;) {

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    d = gData;
    d.time = millis();
    static uint32_t lastTime = 0;
    uint32_t now = micros();

    d.dt = now - lastTime;
    lastTime = now;
    xSemaphoreGive(dataMutex);

    // Safe queue push
    if (xQueueSend(logQueue, &d, 0) != pdTRUE) {
      // queue full → drop
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz logging
  }
}

// --------------------
// LOGGER TASK
// --------------------
void loggerTask(void* param) {
  SensorData d;
  int flushCounter = 0;

  for (;;) {

    // Take ONE item (prevents SD overload)
    if (xQueueReceive(logQueue, &d, portMAX_DELAY) == pdTRUE) {

      // SERIAL
      Serial.print(d.time); Serial.print(",");
      Serial.print(d.ax); Serial.print(",");
      Serial.print(d.ay); Serial.print(",");
      Serial.print(d.az); Serial.print(",");
      Serial.print(d.altitude); Serial.print(",");
      Serial.println(d.velocity);

      // SD WRITE
      xSemaphoreTake(spiMutex, portMAX_DELAY);

      digitalWrite(LORA_CS, HIGH);
      if (logFile) {
        logFile.print(d.time); logFile.print(",");
        logFile.print(d.ax); logFile.print(",");
        logFile.print(d.ay); logFile.print(",");
        logFile.print(d.az); logFile.print(",");
        logFile.print(d.gx); logFile.print(",");
        logFile.print(d.gy); logFile.print(",");
        logFile.print(d.gz); logFile.print(",");
        logFile.print(d.mx); logFile.print(",");
        logFile.print(d.my); logFile.print(",");
        logFile.print(d.mz); logFile.print(",");
        logFile.print(d.altitude); logFile.print(",");
        logFile.print(d.velocity); logFile.print(",");
        logFile.println(d.pressure);
      }

      xSemaphoreGive(spiMutex);
      flushCounter++;
      if (flushCounter >= 10) {
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        if (logFile) logFile.flush();
        xSemaphoreGive(spiMutex);
        flushCounter = 0;
      }
    }
  }
}

// --------------------
// SETUP
// --------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== STABLE LOGGER START ===");

  // I2C
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);

  // SPI
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
  SPI.begin();

  // Mutex
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  spiMutex  = xSemaphoreCreateMutex();

  // SMALL QUEUE (important)
  logQueue = xQueueCreate(20, sizeof(SensorData));

  // Sensors
  lis.begin();
  icm.begin();
  bmp.begin();

  // SD
  if (SD.begin(SD_CS)) {

    String filename = getNextFilename();
    Serial.println("Logging to: " + filename);

    logFile = SD.open(filename, FILE_WRITE);

    if (logFile) {
      logFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,alt,vel,pressure");
      logFile.flush();
    }
  } else {
    Serial.println("SD FAIL");
  }

  // Tasks (SAFE STACK SIZES)
  xTaskCreate(icmTask, "ICM", 1024, NULL, 2, NULL);
  xTaskCreate(bmpTask, "BMP", 1024, NULL, 1, NULL);
  xTaskCreate(magTask, "MAG", 1024, NULL, 1, NULL);
  xTaskCreate(samplerTask, "SAMPLE", 1024, NULL, 1, NULL);
  xTaskCreate(loggerTask, "LOGGER", 2048, NULL, 0, NULL);
}

// --------------------
void loop() {}