#define BuzzerPin A1

void setup()
{
    // setup buzzer 
    pinMode(BuzzerPin, OUTPUT);
}

void loop()
{
    // turn on buzzer
    digitalWrite(BuzzerPin, HIGH);
    // delay 1 second
    delay(1000);
    // turn off buzzer
    digitalWrite(BuzzerPin, LOW);
    // delay 1 second
    delay(1000);
}