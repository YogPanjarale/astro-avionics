#include <Arduino.h>
#include "E32.h"
#include "triggerEjectionCharges.h"
#include "checkEjectionCharges.h"
#include "bmp.h"
#include "imu.h"
#include "buzzer.h"

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
// ejection charge continuity , 0 means no continuity , 1 means continuity
#define ECd_h    (1 << 5) //ejecion charge continuity drogue 
#define ECm_h    (1 << 6) //ejecion charge continuity main
#define ECb_h    (1 << 7) //ejecion charge continuity backup
// ejection charge status , 0 means not fired , 1 means fired
#define ECrd_h    (1 << 8) //ejecion charge status drogue
#define ECrm_h    (1 << 9) //ejecion charge status main
#define ECrb_h    (1 << 10) //ejecion charge status backup
// next 3 (11,12,13) bits show the state of the rocket, 000 means boot, 001 means connection, 010 means calibration, 011 means idle, 100 means flight, 101 means drogue, 110 means parachute, 111 means recovery
#define S10_h    (1 << 11) // state bit 0
#define S11h    (1 << 12) // state bit 1
#define S12_h    (1 << 13) // state bit 2

// binary status register
// 0b0000000000000000 
// bit 0: raspberry pi status
// bit 1: gps status
// bit 2: lora status
// bit 3: bmp status
// bit 4: imu status
// bit 5 ,6 ,7: ejection charge continuity (drogue, main, backup) 
// bit 8,9,10: ejection charge status (drogue, main, backup)
// bit 11,12,13: state of the rocket
uint16_t status = 0b0000000000000000;
// define ejection charge status register


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

void setState(State s){
  state = s;
  // update the status register , set the state bits
// next 3 (11,12,13) bits show the state of the rocket, 000 means boot, 001 means connection, 010 means calibration, 011 means idle, 100 means flight, 101 means drogue, 110 means parachute, 111 means recovery
  switch (s)
  {
  case BOOT:
    status &= ~(S10_h | S11h | S12_h); //000
    break;
  case CONN:
    status &= ~(S10_h | S11h); //001
    status |= S12_h;
    break;
  case CALIB:
    status &= ~(S10_h | S12_h); //010
    status |= S11h;
    break;
  case IDLE:
    status &= ~(S10_h); // 011
    status |= S11h | S12_h;
    break;
  case FLIGHT:
    status &= ~(S11h | S12_h); // 100
    status |= S10_h;
    break;
  case DROUGE:
    status &= ~(S10_h | S11h); // 101
    status |= S12_h;
    break;
  case PARACHUTE:
    status &= ~(S10_h); // 110
    status |= S11h | S12_h;
    break;
  case RECOVERY:
    status |= S10_h | S11h | S12_h; // 111
    break;
  default:
    break;
  }


}

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
  uint16_t statusReg;
};

String packDATA(Data data){
  String binaryStatus = "";
  for (int i = 15; i >= 0; i--) {
    binaryStatus += String((data.statusReg >> i) & 1);
  }
  return "DATA:" + String(data.bmpAltitude) + "," + String(data.imuAltitude) + "," + String(data.pressure) + "," + String(data.latitude) + "," + String(data.longitude) + "," + String(data.accel_x) + "," + String(data.accel_y) + "," + String(data.accel_z) + "," + String(data.vel_bmp) + "," + String(data.vel_imu) + "," + binaryStatus;
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
    setState (CONN);
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
    // this loop runs while the time is less than connect time , or  the raspberry pi and lora module are not connected
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
    }
    // beep buzzer long for 2 seconds
    digitalWrite(BuzzerPin, HIGH);
    delay(2000);
    digitalWrite(BuzzerPin, LOW);
    setState(CALIB);
  }
  while (state == CALIB){
    // initialize the sensors and calibrate them
    // setup the bmp sensor
    if(bmpSensor.begin()){
      status |= BMP_h;
    }
    // setup the imu
    if (setupIMU()){
      status |= IMU_h;
    }
    
    // play calibration start tone
    playCalibrationStartTone(); // takes 1 second
    for (int i = 0; i < 10; i++){
      groundAltitude += bmpSensor.getAltitude();
      delay(10);
    }
    groundAltitude /= 10;

    SerialRaspi.println("GROUND_ALTITUDE:" + String(groundAltitude));
    
    
    // IMU will take 500 samples to calibrate , each 5ms , total 2.5 seconds
    calibrateIMU(500); 
    // play a tone to indicate that the calibration is done
    playCalibrationStartTone(); // takes 1 second

    setState(IDLE);
  }
  while (state == IDLE) {
    // read the data from the sensors
    Data data;
    data.bmpAltitude = bmpSensor.getAltitude();
    data.pressure = bmpSensor.getPressure();
    data.imuAltitude = getHeightIMU();
    data.accel_x = readIMU().accel_x;
    data.accel_y = readIMU().accel_y;
    data.accel_z = readIMU().accel_z;
    data.vel_imu = getVelocityIMU();
    data.vel_bmp = bmpSensor.getVelocity();
    data.statusReg = status;
    // TODO : read the data from the GPS

    // send the data to the raspberry pi
    SerialRaspi.println(packDATA(data));
    // send 
    // check for commands from the raspberry pi
    if((BMP_h & status) && (bmpSensor.getAltitude()-groundAltitude>200) || (IMU_h & status) && (getHeightIMU()>200)){
        setState(FLIGHT);
    }
  }

  while(state == FLIGHT){
    //TODO
  }

}