#include <Arduino.h>
#include "gps.h"

#define GPS_RX 1  // GPS Module RX pin
#define GPS_TX 0  // GPS Module TX pin

void setupGPS(){
    static bool setup_done = false;
    // may be called multiple time , execute only once;
    if (setup_done){
        return;
    }
    // setup the gps module
    Serial0.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

GPSData readGPSData(){
    GPSData data;
    return data;
}

String packGPSDATA(GPSData data){
    char buffer[100];
    Serial0.readBytesUntil('\n',buffer,100);
    // convert buffer to string
    String str = String(buffer);
    return str;
}

