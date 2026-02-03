/*
Continuity Check - 
Main - 
D5 - Power
A0 - Detect
Drogue - 
D4 - Power
D6 - Detect
Backup - 
D3 - Power
D2 - Detect
*/
#define mainDetect A0
#define mainPower D5
#define droguePower D4
#define drogueDetect D6
#define backupPower D3
#define backupDetect D2


/*
return 0 if not connected , 1 if connected
*/
int getContinuityStatus(int detectPin, int powerPin){
    // set the power pin to high
    digitalWrite(powerPin, HIGH);
    delay(10);
    int status = digitalRead(detectPin)== HIGH ? 0 : 1;
    // set the power pin to low
    digitalWrite(powerPin, LOW);
    return status;
}

void setup (){
    // set up the serial communication
    Serial.begin(115200);
    
    // set up the pinmodes pins 
    pinMode(mainPower, OUTPUT);
    pinMode(mainDetect, INPUT);
    pinMode(droguePower, OUTPUT);
    pinMode(drogueDetect, INPUT);
    pinMode(backupPower, OUTPUT);
    pinMode(backupDetect, INPUT);
}

void loop (){
    int mainStatus = getContinuityStatus(mainDetect, mainPower);
    int drogueStatus = getContinuityStatus(drogueDetect, droguePower);
    int backupStatus = getContinuityStatus(backupDetect, backupPower);
    Serial.print("Main Continuity: ");
    Serial.println(mainStatus);
    Serial.print("Drogue Continuity: ");
    Serial.println(drogueStatus);
    Serial.print("Backup Continuity: ");
    Serial.println(backupStatus);
    delay(1000);
}
