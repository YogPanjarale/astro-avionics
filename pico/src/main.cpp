#define __FREERTOS 1

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>

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

#define SPI_CS_LORA 5
#define LORA_RESET 6
#define LORA_DIO0  7

#define GPS_TX 8
#define GPS_RX 9

#define LORA_FREQ 433E6

// --------------------
// FLIGHT STATE
// --------------------
enum class FlightState {
  START,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};

// --------------------
// STRUCTS
// --------------------
struct vector3 {
  float x;
  float y;
  float z;
};

struct SharedData {

  uint32_t timestamp;
  uint32_t dt;

  float BMPA_Temperature;
  float BMPA_Pressure;
  float BMPA_Altitude;
  float BMPA_RelativeAltitude;
  float BMPA_Velocity;

  float BMPB_Temperature;
  float BMPB_Pressure;
  float BMPB_Altitude;
  float BMPB_Velocity;

  vector3 ICM_Accel;
  float ICM_Velocity_Z;
  vector3 ICM_Gyro;
  vector3 ICM_Theta;
  float ICM_Temperature;

  vector3 LIS_Mag;
  float LIS_Temperature;

  float GPS_Latitude;
  float GPS_Longitude;

  FlightState current_state;
};

// --------------------
// OBJECTS
// --------------------
#define MAG_ADDR 0x1E
#define ICM_ADDR 0x68
#define BMP_ADDR 0x77

LIS3MDL lis(Wire1, MAG_ADDR);
ICM20649 icm(Wire1, ICM_ADDR);
BMP bmp(Wire1, BMP_ADDR);
GPS gps(Serial2, GPS_TX, GPS_RX);

File logFile;

// --------------------
// RTOS
// --------------------
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t spiMutex;

QueueHandle_t logQueue;

SharedData gData;

// --------------------
String getNextFilename() {
  for (int i = 1; i < 1000; i++) {
    String name = "/log_" + String(i) + ".csv";
    if (!SD.exists(name)) return name;
  }
  return "/log_max.csv";
}

// --------------------
FlightState randomState() {
  return (FlightState)random(0, 5);
}

// --------------------
// TASKS
// --------------------
void icmTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    icm.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.ICM_Accel = {icm.accelX(), icm.accelY(), icm.accelZ()};
    gData.ICM_Gyro  = {icm.gyroX(), icm.gyroY(), icm.gyroZ()};
    gData.ICM_Temperature = icm.temperature();
    gData.ICM_Velocity_Z = gData.ICM_Accel.z * 0.02f;

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void magTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    lis.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    gData.LIS_Mag = {lis.getMagX(), lis.getMagY(), lis.getMagZ()};
    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void bmpTask(void* param) {
  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bmp.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.BMPA_Altitude = bmp.getAltitude();
    gData.BMPA_RelativeAltitude = bmp.getRelativeAltitude();
    gData.BMPA_Velocity = bmp.getVelocity();
    gData.BMPA_Pressure = bmp.getPressure();
    gData.BMPA_Temperature = bmp.getTemperature();

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void gpsTask(void* param) {

  uint32_t startTime = millis();

  for (;;) {

    if (millis() - startTime < 15000) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    gps.read();

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    if (gps.hasFix()) {
      gData.GPS_Latitude = gps.getLatitude();
      gData.GPS_Longitude = gps.getLongitude();
    }

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --------------------
// SAMPLER (timestamp + dt)
// --------------------
void samplerTask(void* param) {

  SharedData d;
  static uint32_t lastMicros = 0;

  for (;;) {

    uint32_t nowMicros = micros();
    uint32_t nowMillis = millis();

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    d = gData;
    d.timestamp = nowMillis;
    d.dt = nowMicros - lastMicros;
    d.current_state = randomState();

    xSemaphoreGive(dataMutex);

    lastMicros = nowMicros;

    xQueueSend(logQueue, &d, 0);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --------------------
// LOGGER
// --------------------
void loggerTask(void* param) {

  SharedData d;

  for (;;) {

    if (xQueueReceive(logQueue, &d, pdMS_TO_TICKS(100)) == pdTRUE) {

      char buffer[300];

      snprintf(buffer, sizeof(buffer),
        "%lu,%lu,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.6f,%.6f,%d",
        d.timestamp,
        d.dt,
        d.BMPA_Temperature,
        d.BMPA_Pressure,
        d.BMPA_Altitude,
        d.BMPA_RelativeAltitude,
        d.BMPA_Velocity,
        d.ICM_Accel.x,
        d.ICM_Accel.y,
        d.ICM_Accel.z,
        d.ICM_Gyro.x,
        d.ICM_Gyro.y,
        d.ICM_Gyro.z,
        d.LIS_Mag.x,
        d.LIS_Mag.y,
        d.LIS_Mag.z,
        d.GPS_Latitude,
        d.GPS_Longitude,
        (int)d.current_state
      );

      Serial.println(buffer);

      // only bmp for drbugging 
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f\n",
                    d.timestamp,
                    d.BMPA_Temperature,
                    d.BMPA_Pressure,
                    d.BMPA_Altitude,
                    d.BMPA_RelativeAltitude);

      xSemaphoreTake(spiMutex, portMAX_DELAY);
      digitalWrite(SPI_CS_LORA, HIGH);

      if (logFile) logFile.println(buffer);

      xSemaphoreGive(spiMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// --------------------
// LORA
// --------------------
void loraTask(void* param) {

  SharedData d;

  for (;;) {

    if (xQueuePeek(logQueue, &d, pdMS_TO_TICKS(200)) == pdTRUE) {

      char payload[160];

      snprintf(payload, sizeof(payload),
        "%lu,%lu,%.2f,%.2f,%.2f,%.6f,%.6f",
        d.timestamp,
        d.dt,
        d.BMPA_Altitude,
        d.BMPA_RelativeAltitude,
        d.ICM_Velocity_Z,
        d.GPS_Latitude,
        d.GPS_Longitude
      );

      xSemaphoreTake(spiMutex, portMAX_DELAY);
      digitalWrite(SD_CS, HIGH);

      LoRa.beginPacket();
      LoRa.print(payload);
      LoRa.endPacket();

      xSemaphoreGive(spiMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// --------------------
// SETUP
// --------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);

  pinMode(SD_CS, OUTPUT);
  pinMode(SPI_CS_LORA, OUTPUT);

  digitalWrite(SD_CS, HIGH);
  digitalWrite(SPI_CS_LORA, HIGH);

  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
  SPI.begin();

  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  spiMutex  = xSemaphoreCreateMutex();

  logQueue = xQueueCreate(20, sizeof(SharedData));

  lis.begin();
  icm.begin();
  bmp.begin();

  Serial2.setTX(GPS_TX);
  Serial2.setRX(GPS_RX);
  Serial2.begin(9600);
  gps.begin(9600);

  // SD + HEADER ✅
  if (SD.begin(SD_CS)) {

    logFile = SD.open(getNextFilename(), FILE_WRITE);

    if (logFile) {
      logFile.println("time,dt,bmp_temp,bmp_press,bmp_alt,bmp_rel_alt,bmp_vel,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,state");
      logFile.flush();
    }
  }

  // LORA INIT
  xSemaphoreTake(spiMutex, portMAX_DELAY);

  LoRa.setPins(SPI_CS_LORA, LORA_RESET, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(17);
  LoRa.enableCrc();

  xSemaphoreGive(spiMutex);

  // TASKS
  xTaskCreate(icmTask, "ICM", 1024, NULL, 2, NULL);
  xTaskCreate(bmpTask, "BMP", 1024, NULL, 1, NULL);
  xTaskCreate(magTask, "MAG", 1024, NULL, 1, NULL);
  xTaskCreate(samplerTask, "SAMPLE", 1024, NULL, 1, NULL);
  xTaskCreate(loggerTask, "LOGGER", 2048, NULL, 1, NULL);
  xTaskCreate(gpsTask, "GPS", 2048, NULL, 1, NULL);
  xTaskCreate(loraTask, "LORA", 2048, NULL, 1, NULL);
}

void loop() {}