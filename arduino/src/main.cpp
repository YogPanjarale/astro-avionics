#include <Arduino.h>
#include "E32.h"
#include "testEjectionCharges.h"
#include "bmp.h"
#include "imu.h"

#define BMP_SDA A4
#define BMP_SCL A5
BMPSensor bmpSensor(BMP_SDA, BMP_SCL);

// define pins
#define BuzzerPin A1
#define RaspiTX A7
#define RaspiRX A6
#define E32RX A3
#define E32TX A2
#define GPSRX D1
#define GPSTX D0

// define Constants
#define BOOT_TIME 15000    // time to wait for both systems to boot up
#define CONNECT_TIME 15000 // time to wait for the connection to be established

// Define serial ports
HardwareSerial SerialE32(1);   // Use UART1 (A3 RX, A2 TX)
HardwareSerial SerialRaspi(2); // Use UART2 (A6 RX, A7 TX)
HardwareSerial SerialGPS(3); //Use UART3 (D0 TX, D1 RX)
E32Module e32(SerialE32);


// define the status register bits
#define RPI_h    (1 << 0) // is the raspberry pi alive
#define GPS_h    (1 << 1) // is the gps alive
#define LORA_h   (1 << 2) // is the lora alive
#define BMP_h    (1 << 3) // is the bmp alive
#define IMU_h    (1 << 4) //is the imu alive
#define ECd_h    (1 << 5) //ejecion charge drogue
#define ECm_h    (1 << 6) //ejecion charge main
#define ECb_h    (1 << 7) //ejecion charge backup
// binary status register
// 0b00000000
int status = 0b00000000;


enum State {
  BOOT,
  CONN,
  CALIB,
  IDLE,
  FLIGHT,
  DROUGE,
  PARACHUTE,
  RECOVERY,
};

State state = BOOT;

// state variables
float groundAltitude = 0;

// full data struct
struct Data {
  float bmpAltitude;
  float imuAltitude;
  float pressure;
  float latitude;
  float longitude;
  float accel_x;
  float accel_y;
  float accel_z;
  float vel_bmp;
  float vel_imu;
};

String packDATA(Data data){
  return "DATA:" + String(data.bmpAltitude) + "," + String(data.imuAltitude) + "," + String(data.pressure) + "," + String(data.latitude) + "," + String(data.longitude) + "," + String(data.accel_x) + "," + String(data.accel_y) + "," + String(data.accel_z) + "," + String(data.vel_bmp) + "," + String(data.vel_imu);
}

void setup() {
  Serial.begin(115200);

  // initialize the buzzer
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
}

void loop() {
  while(state == BOOT) {
    // beep buzzer for 200ms every second till the end of the boot time
    for(int i = 0; i < BOOT_TIME; i+=1000) {
      digitalWrite(BuzzerPin, HIGH);
      delay(200);
      digitalWrite(BuzzerPin, LOW);
      delay(800);
    }
    state = CONN;
  }
  while (state == CONN){
    // begin communication with the raspberry pi
    SerialRaspi.begin(115200, SERIAL_8N1, RaspiRX, RaspiTX); 

    // begin communication with the e32
    SerialE32.begin(115200, SERIAL_8N1, E32RX, E32TX);

    // begin communication with GPS
    SerialGPS.begin(115200, SERIAL_8N1, GPSRX, GPSTX);


    // connect to raspberry pi
    unsigned long start = millis();
    // this loop runs while the time is less than connect time , and the raspberry pi and lora module are not connected
    while (millis() - start < CONNECT_TIME ||  (!(status & RPI_h) || !(status & LORA_h))){
      // send ping , and wait for pong
      SerialRaspi.println("PING");
      if(SerialRaspi.available()){
        String response = SerialRaspi.readStringUntil('\n');
        if(response == "PONG"){
          status |= RPI_h;
          break;
        }
      }

      // send ping to lora module
      e32.sendMessage("PING");
      if(SerialE32.available()){
        String response = SerialE32.readStringUntil('\n');
        if(response == "PONG"){
          status |= LORA_h;
          break;
        }
      }
    }
    // beep buzzer long for 2 seconds
    digitalWrite(BuzzerPin, HIGH);
    delay(2000);
    digitalWrite(BuzzerPin, LOW);
    state  = CALIB;
  }
  while (state == CALIB){
    // initialize the sensors and calibrate them
    if(bmpSensor.begin()){
      status |= BMP_h;
    }
    for (int i = 0; i < 10; i++){
      groundAltitude += bmpSensor.getAltitude();
      delay(10);
    }
    groundAltitude /= 10;

    SerialRaspi.println("GROUND_ALTITUDE:" + String(groundAltitude));
    
    // TODO : calibrate the IMU
    status |= IMU_h;

    state = IDLE;
  }
  while (state == IDLE) {
    // read the data from the sensors
    Data data;
    data.bmpAltitude = bmpSensor.getAltitude();
    // TODO : read the data from the IMU
    // TODO : read the data from the GPS
    // send the data to the raspberry pi
    SerialRaspi.println(packDATA(data));
    // send 
    // check for commands from the raspberry pi
    if(bmpSensor.getAltitude() > 200)
    {
      state = FLIGHT;
    }

  while(state == FLIGHT)
  {
    SerialE32.println("Flight mode activated!");
    SerialRaspi.println("Flight mode activated!");

    Data data;
    data.bmpAltitude = bmpSensor.getAltitude();

    //get height from imu
    
  }
  }



}