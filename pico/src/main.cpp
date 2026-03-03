#define __FREERTOS 1

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// our libs
#include <BMP.h>
#include <GPS.h>
#include <ICM20649.h>
#include <LIS3MDL.h>
#include <pindefs.h>

// defining sensors 
BMP bmpA(Wire, 0x77);
BMP bmpB(Wire1, 0x77);
ICM20649 icm(Wire);
LIS3MDL lis(Wire);
GPS gps(Serial1, GPS_TX, GPS_RX);

// --------------------
// Shared resource
// --------------------
struct vector3 {
  float x;
  float y;
  float z;
};
struct SharedData
{
  float BMPA_Temperature;
  float BMPA_Pressure;
  float BMPA_Altitude;
  float BMPA_Velocity;
  float BMPB_Temperature;
  float BMPB_Pressure;
  float BMPB_Altitude;
  float BMPB_Velocity;
  vector3 ICM_Accel;
  vector3 ICM_Gyro;
  float ICM_Temperature;
  vector3 LIS_Mag;
  float LIS_Temperature;
  float GPS_Latitude;
  float GPS_Longitude;
};
SemaphoreHandle_t dataMutex;
SharedData sharedData;

volatile int globalCounter = 0;
SemaphoreHandle_t counterMutex;

// --------------------
// Task prototypes
// --------------------
void task1(void* param);
void task2(void* param);
void task3(void* param);
void task4(void* param);
void task5(void* param);

void setup() {
  Serial.begin(115200);
  Wire.setSDA(SDA_A);
  Wire.setSCL(SCL_A);
  Wire1.setSDA(SDA_B);
  Wire1.setSCL(SCL_B);
  while (!Serial);

  Serial.println("Starting FreeRTOS with shared memory...");

  // Create mutex
  counterMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  if (dataMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);
  }

  // Create tasks
  xTaskCreate(task1, "BMPA", 1024, NULL, 1, NULL);
  xTaskCreate(task2, "BMPB", 1024, NULL, 1, NULL);
  xTaskCreate(task3, "ICM", 1024, NULL, 1, NULL);
  xTaskCreate(task4, "LIS", 1024, NULL, 1, NULL);
  xTaskCreate(task5, "GPS", 1024, NULL, 1, NULL);

  // init senors
  bmpA.begin();
  bmpB.begin();
  icm.begin();
  lis.begin();
  gps.begin();

}

void loop() {
  // Not used
}

// --------------------
// Task 1 : BMPA
// --------------------
void task1(void* param) {
  for (;;) {
    bmpA.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.BMPA_Temperature = bmpA.getTemperature();
      sharedData.BMPA_Pressure = bmpA.getPressure();
      sharedData.BMPA_Altitude = bmpA.getAltitude();
      sharedData.BMPA_Velocity = bmpA.getVelocity();
      // inline prinitng
      Serial.print("BMPA Temp: ");
      Serial.print(sharedData.BMPA_Temperature);
      Serial.print(" C, Pressure: ");
      Serial.print(sharedData.BMPA_Pressure);
      Serial.print(" hPa, Altitude: ");
      Serial.print(sharedData.BMPA_Altitude);
      Serial.print(" m, Velocity: ");
      Serial.print(sharedData.BMPA_Velocity);
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// --------------------
// Task 2 : BMPB
// --------------------
void task2(void* param) {
  for (;;) {
    bmpB.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.BMPB_Temperature = bmpB.getTemperature();
      sharedData.BMPB_Pressure = bmpB.getPressure();
      sharedData.BMPB_Altitude = bmpB.getAltitude();
      sharedData.BMPB_Velocity = bmpB.getVelocity();
      Serial.print("BMPB Temp: ");
      Serial.print(sharedData.BMPB_Temperature);
      Serial.print(" C, Pressure: ");
      Serial.print(sharedData.BMPB_Pressure);
      Serial.print(" hPa, Altitude: ");
      Serial.print(sharedData.BMPB_Altitude);
      Serial.print(" m, Velocity: ");
      Serial.print(sharedData.BMPB_Velocity);
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

// --------------------
// Task 3 : ICM20649
// --------------------
void task3(void* param) {
  for (;;) {
    icm.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.ICM_Accel.x = icm.getAccelX();
      sharedData.ICM_Accel.y = icm.getAccelY();
      sharedData.ICM_Accel.z = icm.getAccelZ();
      sharedData.ICM_Gyro.x = icm.getGyroX();
      sharedData.ICM_Gyro.y = icm.getGyroY();
      sharedData.ICM_Gyro.z = icm.getGyroZ();
      sharedData.ICM_Temperature = icm.getTemperature();
      Serial.print("ICM Accel: (");
      Serial.print(sharedData.ICM_Accel.x);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Accel.y);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Accel.z);
      Serial.print(") m/s^2, Gyro: (");
      Serial.print(sharedData.ICM_Gyro.x);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Gyro.y);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Gyro.z);
      Serial.print(") deg/s, Temp: ");
      Serial.print(sharedData.ICM_Temperature);
      Serial.println(" C");
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// --------------------
// Task 4 : LIS3MDL
// --------------------
void task4(void* param) {
  for (;;) {
    lis.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.LIS_Mag.x = lis.getMagX();
      sharedData.LIS_Mag.y = lis.getMagY();
      sharedData.LIS_Mag.z = lis.getMagZ();
      sharedData.LIS_Temperature = lis.getTemperature();
      Serial.print("LIS Mag: (");
      Serial.print(sharedData.LIS_Mag.x);
      Serial.print(", ");
      Serial.print(sharedData.LIS_Mag.y);
      Serial.print(", ");
      Serial.print(sharedData.LIS_Mag.z);
      Serial.print(") uT, Temp: ");
      Serial.print(sharedData.LIS_Temperature);
      Serial.println(" C");
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2500));
  }
}

// --------------------
// Task 5 : GPS
// --------------------
void task5(void* param) {
  for (;;) {
    gps.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.GPS_Latitude = gps.getLatitude();
      sharedData.GPS_Longitude = gps.getLongitude();
      Serial.print("GPS Latitude: ");
      Serial.print(sharedData.GPS_Latitude);
      Serial.print(" deg, Longitude: ");
      Serial.print(sharedData.GPS_Longitude);
      Serial.println(" deg");
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}
