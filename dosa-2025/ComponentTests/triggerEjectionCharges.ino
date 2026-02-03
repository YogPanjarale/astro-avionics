// write test code to trigger relay pins for 1 sec and turn off

/*
Relays -
D9 - Main
D8 - Drogue
D7 - Backup
*/
#define relayMain 9
#define relayDrogue 8
#define relayBackup 7

void setup() {
    // Set relay pins as OUTPUT
    pinMode(relayMain, OUTPUT);
    pinMode(relayDrogue, OUTPUT);
    pinMode(relayBackup, OUTPUT);

    // Ensure relays are OFF initially
    digitalWrite(relayMain, LOW);
    digitalWrite(relayDrogue, LOW);
    digitalWrite(relayBackup, LOW);
}

void triggerRelay(int relayPin) {
    digitalWrite(relayPin, HIGH); // Turn on relay
    delay(1000);                  // Wait 1 second
    digitalWrite(relayPin, LOW);  // Turn off relay
}

void loop() {
    triggerRelay(relayMain);
    delay(1000);  // Small delay before next relay signal

    triggerRelay(relayDrogue);
    delay(1000);

    triggerRelay(relayBackup);
    delay(1000);
}
