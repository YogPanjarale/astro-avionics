#include <Arduino.h>

void setupGPS();

struct GPSData
{
    double lat,lng;
    float alt,vel,dir;
};

GPSData readGPSData();

String packGPSDATA(GPSData data);