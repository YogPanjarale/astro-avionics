Nano Connections:
GPS -
D0 - GPS Tx
D1 - GPS Rx
Relays -
D9 - Main
D8 - Drogue
D7 - Backup
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
BMP390 - 
A4 - SDA
A5 - SCL
D10 - Power
IMU -
A4 - SDA
A5 - SCL
Buzzer - 
A1 - Power
E32 - 
D11 - M0
D12 - M1
A3 - E32 RX
A2 - E32 TX
Raspi - 
A6 - Raspi RX
A7 - Raspi TX

Raspi Connections:
GPS - 
13 - Force on
Relays - 
22 - Backup
27 - Drogue
17 - Main
Leds  - 
25 - Status
24 - Tel
BMP390 - 
SDA (GPIO2) - SDA
SCL (GPIO3) - SCL
Arduino - 
RX - A6
TX - A7

Continuity checking and ejection charge mechanism 

First power on respective power pin 
Delay of few 100 micro sec
Read input from respective read pin 
When detect is high ejection charge is not connected 
When low ejection charge is connected 
Update state 
Set power pin back to low

Check Continuity once every 300-400 cycles 


To deploy ejection charges When certain conditions meet set respective relay pins high

---

## Flight regimes

Use a flight state variable and a switch case in loop to run the correct functions for each flight regime

ex:

enum State = {BOOT, READY, FLIGHT, DESCENT, LANDING, FAILURE}

State flightstate = BOOT

void loop(){
    switch BOOT:
    switch READY:
    .....
}

## Sensor states

Use a state variable(integer) to get and set states of individual sensors

#define RPI     (1 << 0)
#define GPS     (1 << 1)
#define LORA    (1 << 2)
#define BMP     (1 << 3)
#define IMU     (1 << 4)

int sensorStatus = 0         // This is a global variable

// To manipulate and check statuses

sensorStatus |= RPI          // Set status to true 
sensorStatus &= ~LORA        // Set status to false
sensorStatus ^= BMP          // Toggle status
(sensorStatus & IMU) != 0    // Get if a status is true


## Data being sent

to e32 we can sent 56 bytes at 9600
we send 42 char array.
