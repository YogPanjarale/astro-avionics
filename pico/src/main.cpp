#define __FREERTOS 1

#include <Arduino.h>
#include "FreeRTOS.h"

void task1(void* param) {
  while (true) {
    Serial.println("Task 1 is running");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task2(void* param) {
  while (true) {
    Serial.println("Task 2 is running");
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println("Hello, world!");
  xTaskCreate(task1, "Task1", 1024, NULL, 1, NULL);
  xTaskCreate(task2, "Task2", 1024, NULL, 1, NULL);
}

void loop() {
  delay(1000);
}
