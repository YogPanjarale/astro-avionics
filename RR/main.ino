#include <Wire.h>

#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

#include <SD.h>
#include <SPI.h>

#define chipSelect D5 
#define sdaPin A4
#define sclPin A5



void setup() {
  Serial.begin(115200);

  // BMP init
  Wire.begin(sdaPin, sclPin);
  if (!bmp.begin_I2C(119)){
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X); // new value should be updated every 40ms (25Hz)
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);  // new value should be updated every 40ms (25Hz)

  // SD init
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)){
    Serial.println("initialization failed!");

  }
  Serial.println("initialization done.");
}

void loop() {
  // put your main code here, to run repeatedly:

}
