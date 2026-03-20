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
#include "GPS.h"

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

#define GPS_TX 8
#define GPS_RX 9

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

GPS gps(Serial2, GPS_TX, GPS_RX);

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

  float lat;
  float lon;
  float gps_alt;
  bool gps_fix;

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

void gpsTask(void* param) {

  uint32_t startTime = millis();

  for (;;) {

    // Warmup 15 sec
    if (millis() - startTime < 15000) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Read GPS
    gps.read();

    // Update shared struct
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    if (gps.hasFix()) {
      gData.lat = gps.getLatitude();
      gData.lon = gps.getLongitude();
      gData.gps_alt = gps.getAltitude();
      gData.gps_fix = true;
    } else {
      gData.lat = 0;
      gData.lon = 0;
      gData.gps_alt = 0;
      gData.gps_fix = false;
    }

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
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

    if (xQueueReceive(logQueue, &d, pdMS_TO_TICKS(100)) == pdTRUE) {

      // SERIAL
      char buffer[192];

      snprintf(buffer, sizeof(buffer),
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d",
        d.time,
        d.ax, d.ay, d.az,
        d.gx, d.gy, d.gz,
        d.mx, d.my, d.mz,
        d.altitude,
        d.velocity,
        d.pressure,
        d.lat,
        d.lon,
        d.gps_alt,
        d.gps_fix
      );

      Serial.println(buffer);
      // SD
      xSemaphoreTake(spiMutex, portMAX_DELAY);

      digitalWrite(LORA_CS, HIGH);

      char buffersd[192];

      int len = snprintf(buffersd, sizeof(buffer),
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d\n",
        d.time,
        d.ax, d.ay, d.az,
        d.gx, d.gy, d.gz,
        d.mx, d.my, d.mz,
        d.altitude,
        d.velocity,
        d.pressure,
        d.lat,
        d.lon,
        d.gps_alt,
        d.gps_fix
      );

      if (logFile) {
        logFile.write((uint8_t*)buffersd, len);
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

    vTaskDelay(pdMS_TO_TICKS(10));  // 🔥 VERY IMPORTANT
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
  Serial.println("Init LIS...");
  lis.begin();
  Serial.println("Init ICM...");
  icm.begin();
  Serial.println("Init BMP...");
  bmp.begin();
  Serial.println("Sensors OK");
  Serial2.setTX(GPS_TX);
  Serial2.setRX(GPS_RX);
  Serial.println("GPS start");

  Serial2.begin(9600);
  if (Serial2.available()) {
    Serial.println("GPS DATA FLOWING");
  }
  gps.begin(9600);  // now only starts logic, not hardware
  Serial.println("GPS OK");

  // SD
  if (SD.begin(SD_CS)) {

    String filename = getNextFilename();
    Serial.println("Logging to: " + filename);

    logFile = SD.open(filename, FILE_WRITE);

    if (logFile) {
      logFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,alt,vel,pressure,lat,lon,gps_alt,gps_fix");
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
  xTaskCreate(gpsTask, "GPS", 2048, NULL, 1, NULL);
}

// --------------------
void loop() {}