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

void triggerRelay(int relayPin,int time)
{
    digitalWrite(relayPin, HIGH); // Turn on relay
    delay(time);                  // Wait 1 second
    digitalWrite(relayPin, LOW);  // Turn off relay
}

void triggerMainEjectionCharges(int time)
{
    // Trigger main ejection charges
    triggerRelay(relayMain, time);
}

/**
 * @brief Triggers the drogue ejection charges.
 * 
 * This function activates the relay responsible for the drogue ejection charges
 * for a specified duration of time.
 * 
 * @param time The duration (in milliseconds) for which the relay should be activated.
 */
void triggerDrogueEjectionCharges(int time)
{
    // Trigger drogue ejection charges
    triggerRelay(relayDrogue, time);
}

void triggerBackupEjectionCharges(int time)
{
    // Trigger backup ejection charges
    triggerRelay(relayBackup, time);
}
