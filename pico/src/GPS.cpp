#include "GPS.h"
#include <stdlib.h>
#include <string.h>

// ----------------------

GPS::GPS(Stream& serial) {
  _serial = &serial;
}

// ----------------------

bool GPS::begin(uint32_t baud) {
  return true;
}

// ----------------------

void GPS::update() {
  while (_serial->available()) {
    char c = _serial->read();

    if (c == '\n') {
      line[index] = 0;
      parseLine(line);
      index = 0;
    } else {
      if (index < sizeof(line) - 1) {
        line[index++] = c;
      }
    }
  }
}

// ----------------------
// Tiny GPGGA parser

void GPS::parseLine(char* nmea) {

  if (strstr(nmea, "$GPGGA") == nullptr) return;

  char* p = strtok(nmea, ",");

  int field = 0;
  while (p != nullptr) {
    field++;

    switch (field) {

      case 2: {   // UTC time hhmmss.sss
        float t = atof(p);
        hour   = (int)(t / 10000);
        minute = (int)(t / 100) % 100;
        second = (int)t % 100;
      } break;

      case 3: latitude = atof(p) / 100.0; break;
      case 4: if (*p == 'S') latitude *= -1; break;

      case 5: longitude = atof(p) / 100.0; break;
      case 6: if (*p == 'W') longitude *= -1; break;

      case 7: fix = atoi(p) > 0; break;

      case 10: altitude = atof(p); break;
    }

    p = strtok(nullptr, ",");
  }
}

// ----------------------
// Getters

float GPS::getLatitude()  { return latitude; }
float GPS::getLongitude() { return longitude; }
float GPS::getAltitude()  { return altitude; }
float GPS::getSpeed()     { return speed; }
float GPS::getHeading()   { return heading; }

int GPS::getHour()   { return hour; }
int GPS::getMinute() { return minute; }
int GPS::getSecond() { return second; }

bool GPS::hasFix() { return fix; }
