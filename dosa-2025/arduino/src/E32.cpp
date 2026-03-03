#include "E32.h"
#include <Arduino.h>
E32Module::E32Module(HardwareSerial &serial) : e32Serial(serial) {}

void E32Module::setup() {
    

    // Setup M0 and M1 for Normal Transmission Mode
    pinMode(e32M0, OUTPUT);
    pinMode(e32M1, OUTPUT);
    digitalWrite(e32M0, LOW);
    digitalWrite(e32M1, LOW);
}

void E32Module::sendMessage(String message) {
    int length = message.length();  // Get length of the string
    byte byteArray[length + 1];     // Create a byte array (+1 for null termination)
    
    message.getBytes(byteArray, length + 1); // Convert String to byte array

    //Serial.print("Broadcasting: ");
    //Serial.println(message);  // Debugging output

    e32Serial.write(byteArray, length); 
}
