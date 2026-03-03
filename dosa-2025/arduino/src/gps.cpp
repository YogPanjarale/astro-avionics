#include <Arduino.h>
#include "gps.h"
void setupGPS(){
    static bool setup_done = false;
    // may be called multiple time , execute only once;
}

GPSData readGPSData(){
    // TODO implement 
    GPSData data;
    return data;
}

String packGPSDATA(GPSData data){
    // TODO: implement
    return "TODO gps packing will be implemented ";
}
