#ifndef E32_H
#define E32_H

#include <Arduino.h>

#define e32RX A3
#define e32TX A2
#define e32M0 D11
#define e32M1 D12

class E32Module
{
public:
    E32Module(HardwareSerial &serial); // Constructor
    void setup();         // setup the module
    void sendMessage(String message);  // Send a string message

private:
    HardwareSerial &e32Serial; // Reference to hardware serial
};

#endif