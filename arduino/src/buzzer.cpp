#include "buzzer.h"
#include <Arduino.h>
#ifndef BUZZER_H
#define BUZZER_H




// play _ _ _ . for 1 second , where _ is 200ms (100ms on , 100 off )and . is 400ms(200ms on , 200ms off)
void playCalibrationStartTone()
{
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
    delay(100);
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
    delay(100);
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
    delay(100);
    digitalWrite(BuzzerPin, HIGH);
    delay(200);
    digitalWrite(BuzzerPin, LOW);
    delay(200); // unnecessary delay , can be removed to reduce the time to 800ms
}

// play . _ _ _  for 1 second , where _ is 200ms (100ms on , 100 off )and . is 400ms(200ms on , 200ms off)
void playCalibrationEndTone()
{
    digitalWrite(BuzzerPin, HIGH);
    delay(200);
    digitalWrite(BuzzerPin, LOW);
    delay(200);
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
    delay(100);
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
    delay(100);
    digitalWrite(BuzzerPin, HIGH);
    delay(100);
    digitalWrite(BuzzerPin, LOW);
    delay(100); // unnecessary delay , can be removed to reduce the time to 900ms
}
#endif