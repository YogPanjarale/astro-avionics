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

// FIX 1: Separate queues for logger and LoRa
// Previously a single logQueue was shared between loggerTask (xQueueReceive)
// and loraTask (xQueuePeek). xQueuePeek does not remove items, so loraTask
// only transmitted when it happened to peek before the logger drained the queue,
// causing intermittent LoRa transmission. Now each task has its own queue.
QueueHandle_t logQueue;
QueueHandle_t loraQueue;

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

  // FIX 2: Proper velocity integration using += accumulation
  // Previously: ICM_Velocity_Z = accelZ * 0.02f  (instantaneous a*dt, not integrated)
  // Now:        ICM_Velocity_Z += accelZ * 0.02f  (running sum = true integration)
  float velocityZ = 0.0f;

  for (;;) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    icm.read();
    xSemaphoreGive(i2cMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.ICM_Accel     = {icm.accelX(), icm.accelY(), icm.accelZ()};
    gData.ICM_Gyro      = {icm.gyroX(),  icm.gyroY(),  icm.gyroZ()};
    gData.ICM_Temperature = icm.temperature();

    velocityZ += icm.accelZ() * 0.02f;   // integrate: v += a * dt
    gData.ICM_Velocity_Z = velocityZ;

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

    gData.BMPA_Altitude         = bmp.getAltitude();
    gData.BMPA_RelativeAltitude = bmp.getRelativeAltitude();
    gData.BMPA_Velocity         = bmp.getVelocity();
    gData.BMPA_Pressure         = bmp.getPressure();
    gData.BMPA_Temperature      = bmp.getTemperature();

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
      gData.GPS_Latitude  = gps.getLatitude();
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

    d               = gData;
    d.timestamp     = nowMillis;
    d.dt            = nowMicros - lastMicros;
    d.current_state = randomState();

    xSemaphoreGive(dataMutex);

    lastMicros = nowMicros;

    // Periodically print stack high water marks — values under ~200 words
    // mean that task needs a larger stack size in xTaskCreate.
    static uint32_t lastStackCheck = 0;
    if (nowMillis - lastStackCheck > 5000) {
      lastStackCheck = nowMillis;
      Serial.printf("STACK HWM — SAMPLE:%u LOGGER:%u LORA:%u ICM:%u BMP:%u MAG:%u GPS:%u\n",
        uxTaskGetStackHighWaterMark(NULL),
        uxTaskGetStackHighWaterMark(xTaskGetHandle("LOGGER")),
        uxTaskGetStackHighWaterMark(xTaskGetHandle("LORA")),
        uxTaskGetStackHighWaterMark(xTaskGetHandle("ICM")),
        uxTaskGetStackHighWaterMark(xTaskGetHandle("BMP")),
        uxTaskGetStackHighWaterMark(xTaskGetHandle("MAG")),
        uxTaskGetStackHighWaterMark(xTaskGetHandle("GPS"))
      );
    }

    // FIX 7: Warn if logQueue is full so dropped samples are visible in Serial output.
    // loraQueue uses silent drop intentionally — stale telemetry is worthless anyway.
    if (xQueueSend(logQueue, &d, 0) != pdTRUE) {
      Serial.println("WARN: logQueue full, sample dropped");
    }
    xQueueSend(loraQueue, &d, 0);

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

      // debug: BMP only
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f\n",
                    d.timestamp,
                    d.BMPA_Temperature,
                    d.BMPA_Pressure,
                    d.BMPA_Altitude,
                    d.BMPA_RelativeAltitude);

      xSemaphoreTake(spiMutex, portMAX_DELAY);

      // FIX 3: Removed spurious digitalWrite(SPI_CS_LORA, HIGH) that was here before.
      // Manually toggling LoRa CS inside the logger is wrong — the LoRa library owns
      // that pin. Only deassert the SD CS before touching the SD card.
      digitalWrite(SD_CS, HIGH);   // deassert SD CS before logger writes (safety guard)

      if (logFile) {
        logFile.println(buffer);
        // FIX 4: Flush after every write so data survives a power cut mid-flight.
        // The original code only flushed the header line in setup(); all subsequent
        // writes lived in the library buffer and would be lost on power loss.
        logFile.flush();
      }

      xSemaphoreGive(spiMutex);
    }
    // FIX 6: Removed vTaskDelay(10) that was here unconditionally.
    // The xQueueReceive above already blocks up to 100ms when the queue is empty,
    // so the extra delay was capping logger throughput to ~50 writes/sec and
    // adding unnecessary latency between SD writes when the queue had backlog.
  }
}

// --------------------
// LORA
// --------------------
void loraTask(void* param) {

  SharedData d;

  for (;;) {

    // FIX 1 (continued): Use xQueueReceive on dedicated loraQueue instead of
    // xQueuePeek on the shared logQueue. xQueuePeek never removed items, so this
    // task only fired when it raced ahead of the logger — causing rare, intermittent
    // transmissions. Now loraTask reliably gets every sample from its own queue.
    if (xQueueReceive(loraQueue, &d, pdMS_TO_TICKS(200)) == pdTRUE) {

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

      // Deassert SD CS so it doesn't interfere with LoRa on the shared SPI bus
      digitalWrite(SD_CS, HIGH);

      LoRa.beginPacket();
      LoRa.print(payload);
      // FIX 5: Non-blocking transmit — endPacket(true) returns immediately and
      // lets the radio finish in the background. The old endPacket() blocked for
      // ~50-100ms at SF7 while holding spiMutex, starving loggerTask of the SD
      // bus and causing the "works for a few seconds, then stops" burst pattern.
      LoRa.endPacket(true);

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

  pinMode(SD_CS,       OUTPUT);
  pinMode(SPI_CS_LORA, OUTPUT);

  digitalWrite(SD_CS,       HIGH);
  digitalWrite(SPI_CS_LORA, HIGH);

  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
  SPI.begin();

  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  spiMutex  = xSemaphoreCreateMutex();

  // FIX 1 (continued): Create the separate LoRa queue.
  // loraQueue is smaller (5 items) since LoRa is slow and older packets are
  // less useful than fresh ones if the queue backs up.
  logQueue  = xQueueCreate(20, sizeof(SharedData));
  loraQueue = xQueueCreate(5,  sizeof(SharedData));

  lis.begin();
  icm.begin();
  bmp.begin();

  Serial2.setTX(GPS_TX);
  Serial2.setRX(GPS_RX);
  Serial2.begin(9600);
  gps.begin(9600);

  // SD + HEADER
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
  xTaskCreate(icmTask,     "ICM",    1024, NULL, 2, NULL);
  xTaskCreate(bmpTask,     "BMP",    1024, NULL, 1, NULL);
  xTaskCreate(magTask,     "MAG",    1024, NULL, 1, NULL);
  xTaskCreate(samplerTask, "SAMPLE", 1024, NULL, 1, NULL);
  xTaskCreate(loggerTask,  "LOGGER", 2048, NULL, 1, NULL);
  xTaskCreate(gpsTask,     "GPS",    2048, NULL, 1, NULL);
  xTaskCreate(loraTask,    "LORA",   2048, NULL, 1, NULL);
}

void loop() {}