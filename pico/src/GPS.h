#pragma once
#include <Arduino.h>

class GPS {
public:
  GPS(Stream& serial);

  bool begin(uint32_t baud = 9600);

  void update();   // call often

  // Position
  float getLatitude();
  float getLongitude();
  float getAltitude();

  // Motion
  float getSpeed();
  float getHeading();

  // Time (UTC)
  int getHour();
  int getMinute();
  int getSecond();

  bool hasFix();

private:
  Stream* _serial;

  // cached data
  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
  float speed = 0;
  float heading = 0;

  int hour = 0;
  int minute = 0;
  int second = 0;

  bool fix = false;

  // parser
  char line[120];
  uint8_t index = 0;

  void parseLine(char* nmea);
};
