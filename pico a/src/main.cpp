#include <Arduino.h>
#include "continuity.h"
#include "BMP.h"
#include "sdlogger.h"

// Flightmode enum def
enum class FlightState {
    GROUND,
    FLIGHT,
    DESCENT
};

volatile FlightState flightState = FlightState::GROUND;

// Shared data def
volatile float altitude = 0;
volatile float velocity = 0;
volatile bool mcont = false;
volatile bool bcont = false;

void setup(){
    contSetup();
}

void loop(){ // flight logic (core 0)

    logger.logSample();

    switch(flightState){

        case FlightState::GROUND:
        break;

        case FlightState::FLIGHT:
        break;

        case FlightState::DESCENT:
        break;
    }
}

void setup1(){ // core 1 setup

    logger.begin();

    logger.logContinuity(mcont, bcont);
}

void loop1(){ // SD card write (core 1)

    logger.flush();

    delay(200);
}