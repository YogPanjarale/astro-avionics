/*
E32 - 
D11 - M0
D12 - M1
A3 - E32 RX
A2 - E32 TX
*/
#define e32RX A3
#define e32TX A2
#define e32M0 D11
#define e32M1 D12
SoftwareSerial e32Serial(e32TX, e32RX); // (TX, RX) syntax for SoftwareSerial

void setup(){
    // setup the serial communication
    Serial.begin(115200);
    e32Serial.begin(9600);  // Set baud rate of E32
    // setup E32
    pinMode(e32M0, OUTPUT);
    pinMode(e32M1, OUTPUT);
    
    digitalWrite(e32M0, LOW);
    digitalWrite(e32M1, LOW);
}

// This function sends a message(byte array) to the E32 module
void sendMessage(byte message[], int length){
    e32Serial.write(message, length);
}

void loop(){
    Serial.println("Sending message");
    byte message[] = {0x01, 0x02, 0x03};
    sendMessage(message, sizeof(message));

    delay(1000);
}