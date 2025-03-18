#include "triggerEjectionCharges.h"
#include <Arduino.h>

void setupRelays()
{
    // Set relay pins as OUTPUT
    pinMode(relayMain, OUTPUT);
    pinMode(relayDrogue, OUTPUT);
    pinMode(relayBackup, OUTPUT);

    // Ensure relays are OFF initially
    digitalWrite(relayMain, LOW);
    digitalWrite(relayDrogue, LOW);
    digitalWrite(relayBackup, LOW);
}

void triggerRelay(int relayPin)
{
    digitalWrite(relayPin, HIGH); // Turn on relay
    delay(1000);                  // Wait 1 second
    digitalWrite(relayPin, LOW);  // Turn off relay
}

void triggerMainEjectionCharges()
{
    // Trigger main ejection charges
    triggerRelay(relayMain);
}
void triggerDrogueEjectionCharges()
{
    // Trigger drogue ejection charges
    triggerRelay(relayDrogue);
}
void triggerBackupEjectionCharges()
{
    // Trigger backup ejection charges
    triggerRelay(relayBackup);
}
