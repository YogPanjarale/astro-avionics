#include <Arduino.h>
#include "E32.h"
#include "triggerEjectionCharges.h"
#include "checkEjectionCharges.h"
#include "bmp.h"
#include "gps.h"
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
#define SAFE_PARACHUTE_VEL -30 // safe velocity below which we can deoply parachute
#define BACKUP_VEL -45

#define SerialRaspi Serial2
#define SerialE32 Serial1
#define SerialGPS Serial0
// Define serial ports
// HardwareSerial SerialE32(1);   // Use UART1 (A3 RX, A2 TX)
// HardwareSerial SerialRaspi(2); // Use UART2 (A6 RX, A7 TX)
// HardwareSerial SerialGPS(3); //Use UART3 (D0 TX, D1 RX)
E32Module e32(SerialE32);


// define the status register bits
#define RPI_h    (1 << 0) // is the raspberry pi alive
#define GPS_h    (1 << 1) // is the gps on
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
// is tip over detected is 14th bit
#define TIP_OVER (1 << 14)
// if the drouge delopyment is confirmed
#define DROGUE_DEPLOYED (1 << 15)
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
// bit 14: TIP_OVER
// bit 15: DROGUE_DEPLOYED
uint16_t status = 0b0000000000000000;
// define ejection charge status register


enum State {
  BOOT,
  CONN,
  CALIB,
  IDLE,
  FLIGHT,
  DROGUE,
  PARACHUTE,
  RECOVERY,
};

State state = BOOT;

void setState(State s){
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
  case DROGUE:
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
  state = s;


}


// state variables
float groundAltitude = 0;
unsigned long timeof_tip_over = 0;

// flight data struct
struct Data {
  unsigned long time;
  float bmpAltitude;
  float imuAltitude;
  float pressure;
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
  return String(data.bmpAltitude) + "," + String(data.imuAltitude) + "," + String(data.pressure) + ","  + String(data.accel_x) + "," + String(data.accel_y) + "," + String(data.accel_z) + "," + String(data.vel_bmp) + "," + String(data.vel_imu) + "," + binaryStatus;
}

// checks all ejection charge continuity and updates status
// checks only once 200 time it is called or force is true , resets counter when force is called
// takes 10*3 ms to execute
void checkAllEjectionChargeContinuity(bool force = false){
  static int count = 0;
  if (count<200&& !force){
    count ++ ;
    return;
  }else{
    count = 0;
  }
  if (checkMainEjectionCharges()){
    status |= ECm_h;
  }else {
    status &= ~ECm_h;
  }
  if (checkBackupEjectionCharges()){
    status |= ECb_h;
  }
  else {
    status &= ~ECb_h;

  }
  if (checkDrogueEjectionCharges()){
    status |= ECd_h;
  }
  else {
    status &= ~ECd_h;

  }
}

/*
we keep the averaging the ax , ay , az with the readings we recive, and if there is more than 40% change, we return 1;
we ignore change in first 10 readings
*/
// int checkTipOver(float ax,float ay , float az){
//   // static float aax = 0, aay=0, aaz =0;
//   // static unsigned long count = 0;
//   // if (count>10){
//   //   // check if anything has changed more then 30% from average
//   //   float dx = abs((aax-ax)/aax) , dy  = abs((aay - ay)/aay) , dz = abs((aaz-az)/aaz);
//   //   if (dx > 0.3 || dy >0.3||dx >0.3)
//   //   {
//   //     return 1;
//   //   }
//   // }
//   // // updating averages
//   //  aax = (count == 0) ? ax : (aax * count + ax) / (count + 1);
//   //  aay = (count == 0) ? ay : (aay * count + ay) / (count + 1);
//   //  aaz = (count == 0) ? az : (aaz * count + az) / (count + 1);

//   //  count++; // increment count


// }



Data updateDataWithoutGPS(){
  Data data;
  data.time = millis();
  data.bmpAltitude = bmpSensor.getAltitude();
  data.pressure = bmpSensor.getPressure();
  data.imuAltitude = getHeightIMU();
  IMUReading reading_i = readIMU();
  data.accel_x = reading_i.accel_x;
  data.accel_y = reading_i.accel_y;
  data.accel_z = reading_i.accel_z;
  data.vel_imu = getVelocityIMU();
  data.vel_bmp = bmpSensor.getVelocity();
  checkAllEjectionChargeContinuity();
  data.statusReg = status;
  return data;
}

void setup() {
  Serial.begin(115200);
  Serial.println("BOOT start");

  // initialize the buzzer
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
}

void serialBeginStuff(bool force = false){
  static bool setup_done = false;
  if (force){
    setup_done = false;
  }
  if (setup_done){
    return;
  }
  // begin communication with the raspberry pi
  SerialRaspi.begin(115200, SERIAL_8N1, RaspiRX, RaspiTX); 

  // begin communication with the e32
  e32.setup();
  SerialE32.begin(115200, SERIAL_8N1, E32RX, E32TX);

  setup_done = true;
}

void sendAllSerial(String data){
  // possible error , print without initializing serial
  Serial.println("Data: ");
    Serial.println(data); // USB debugging 
    SerialRaspi.println(data); // Raspi logging
    e32.sendMessage(data); // Telemetry
}

void loop() {
  while(state == BOOT) {
    Serial.println("BOOT");
    // beep buzzer for 200ms every second till the end of the boot time
    for(int i = 0; i < BOOT_TIME; i+=1000) {
      digitalWrite(BuzzerPin, HIGH);
      delay(200);
      digitalWrite(BuzzerPin, LOW);
      delay(800);
      Serial.println(i); 
    }
    setState (CONN);
    Serial.println("Entering Conn");
  }
  while (state == CONN){
    // it will call it only once ;
    serialBeginStuff();
    Serial.println("Serial Stuff Begin");

    // connect to raspberry pi
    unsigned long start = millis();

    // this loop runs while the time is less than connect time , or  the raspberry pi and lora module are not connected 
    while (((millis() - start) < CONNECT_TIME )&& (!(status & RPI_h))){
      // send ping , and wait for pong
      SerialRaspi.println("PING");
      if(SerialRaspi.available()){
        String response = SerialRaspi.readStringUntil('\n');
        if(response == "PONG"){
          status |= RPI_h;
          break;
        }
      }

      if (SerialE32.available()){
        e32.sendMessage("PING");
        status |= LORA_h;
      }

      // delay 100ms
      Serial.print(".");
      Serial.println(millis() - start );
      digitalWrite(BuzzerPin, HIGH);
      delay(50);
      digitalWrite(BuzzerPin, LOW);
      delay(50);

    }
    Serial.println("Connect end");
    // print if raspi connect was fail
    if(!(status&RPI_h)){
      Serial.println("Raspi Connect fail");
    }
    // beep buzzer long for 2 seconds
    digitalWrite(BuzzerPin, HIGH);
    delay(2000);
    digitalWrite(BuzzerPin, LOW);
    setState(CALIB);
    sendAllSerial("Connection Established");
    sendAllSerial("Starting Calibration");
  }
  while (state == CALIB){
    // initialize the sensors and calibrate them
    // setup the bmp sensor
    if(bmpSensor.begin()){
      status |= BMP_h;
    }
    else{
      // inform all
      sendAllSerial("BMP Sensor not working");
    }
    // setup the imu
    if (setupIMU()){
      status |= IMU_h;
    }
    else {
      // inform all
      sendAllSerial("IMU not working");
    }
    
    // play calibration start tone
    playCalibrationStartTone(); // takes 1 second
    groundAltitude = bmpSensor.getAltitude(); // already oversampling 8x , not doing multiple loop 

    sendAllSerial("GROUND_ALTITUDE:" + String(groundAltitude));
    
    
    // IMU will take 500 samples to calibrate , each 5ms , total 2.5 seconds
    calibrateIMU(500); 

    // we also check ejection Charge Continutity
    checkAllEjectionChargeContinuity(true);
    // play a tone to indicate that the calibration is done
    playCalibrationStartTone(); // takes 1 second

    setState(IDLE);
  }
  while (state == IDLE) {
    // read the data from the sensors
    Data data = updateDataWithoutGPS();
    
    // pack data
    String packed_data = packDATA(data);
    // send All Serial
    sendAllSerial(packed_data);

    if((BMP_h & status) && (bmpSensor.getAltitude()-groundAltitude>200) || (IMU_h & status) && (getHeightIMU()>200)){
        setState(FLIGHT);
        sendAllSerial("Entering Flight Mode");
    }
    // do 50ms delay
    delay(50);
  }
  /*
  In Flight Mode we only take check bmp altitude and imu Height , IMU tipover
  and buffer that data , and send data
  */
  while(state == FLIGHT){
    // update Data 
    // read the data from the sensors
    Data data = updateDataWithoutGPS();
    
    // pack data
    String packed_data = packDATA(data);
   
    // send All Serial
    sendAllSerial(packed_data);

    
    bool s1 = (data.vel_bmp < 1) && (IMU_h & status) ; // if it is alive , and the velocity is less than 1 m/s
    bool s2 = isRocketTippingOver() ;
    bool s3 = (data.vel_imu < 1) &&  (BMP_h & status); // if it is alive , and the velocity is less than 1 m/s

    bool apogee_detected  = false;
    if(apogee_detected){
      
      status |= TIP_OVER;
      timeof_tip_over = millis();
      // send tip over message to raspi
     String tip_over_message = "APOGEE DETECTED: " + String(s1) + "," + String(s2) + "," + String(s3) + "," + String(timeof_tip_over);
      sendAllSerial(tip_over_message); 
      // update continutity status to make things ready for drogue 
      checkAllEjectionChargeContinuity(true);
      setState(DROGUE);
    }
  }
  // enter drogue mode just after apogee detection
  while (state == DROGUE)
  {

    // wait for half second after tip over , and deploy drogue.
    if (millis()-timeof_tip_over > 500){
      triggerDrogueEjectionCharges(2000);
      delay(100); // wait for 100ms and check if deployment was success
      int status_d = checkDrogueEjectionCharges();
      if (!status_d){
        // drogue deoplyment is confirmed
        status |= ECrd_h; // update ejection charge drogue deployment to 1
        status &= ~ECd_h; // update ejection charge drogue continuity to 0
        sendAllSerial("DROGUE Deployed, Enter Parachute mode");
        setState(PARACHUTE);
        status |= DROGUE_DEPLOYED; // update drogue deployment status
      }
      else {

        // pray and enter parachute mode
        sendAllSerial("DROGUE Deploy FAILED , Entering parachute mode");
        setState(PARACHUTE);
        
      }
    }
   
  }
  while(state == PARACHUTE){
    // we send all data , turn on gps , and wait for height to reach below 500m , and deploy parachute , if it does not deploy we try backup
    Data data = updateDataWithoutGPS();
    // > because minus
    if (data.bmpAltitude < 500  || data.vel_bmp < SAFE_PARACHUTE_VEL){
      // we try to deoply parachute
      triggerMainEjectionCharges(2000);
      state = RECOVERY;
      /*
      triggerMainEjectionCharges(2000);
      delay(100); // wait for 100ms and check if deployment was success
      int status_d = checkMainEjectionCharges();
      if (!status_d){
        // main deoplyment is confirmed
        status |= ECrm_h; // update ejection charge main deployment to 1
        status &= ~ECm_h; // update ejection charge main continuity to 0
        SerialRaspi.println("MAIN Parachute Deployed, Enter Recovery");
        e32.sendMessage("MAIN parachue Deployed, Enter Recovery");
        setState(RECOVERY);
      }
      else {
        SerialRaspi.println("MAIN Parachute Deploy FAILED , trying backup ");
        e32.sendMessage("MAIN parachue Deploy FAILED , trying backup");
        // try backup
        triggerBackupEjectionCharges(2000);
        delay(100); // wait for 100ms and check if deployment was success
        int status_d = checkBackupEjectionCharges();
        if (!status_d){
          // backup deoplyment is confirmed
          status |= ECrb_h; // update ejection charge backup deployment to 1
          status &= ~ECb_h; // update ejection charge backup continuity to 0
          SerialRaspi.println("BACKUP Parachute Deployed, Enter Recovery");
          e32.sendMessage("BACUKUP parachue Deployed, Enter Recovery");
          setState(RECOVERY);
        }
        else {
          // pray
        }
      }
      */
    }
    // read gps 
    GPSData gps_d = readGPSData();

     // pack data
     String packed_data = packDATA(data) + packGPSDATA(gps_d);
     // send the data to the raspberry pi
     SerialRaspi.println(packed_data);
     // send the data to Telemetry 
     e32.sendMessage(packed_data);

     // delay 1 sec if vel is <safe Parachute vel
     if(data.vel_bmp<SAFE_PARACHUTE_VEL){
      delay(1000);
     } 
    }
    while (state== RECOVERY)
    {
      
      
      Data data = updateDataWithoutGPS();
      
    // do other things if vel is high
    if(data.vel_bmp < BACKUP_VEL || data.vel_imu < BACKUP_VEL) // If velocity is very high we trigger backup
      triggerBackupEjectionCharges(2000); // Pray after this cuz yes.

     // read gps 
     GPSData gps_d = readGPSData();

     // pack data
     String packed_data = packDATA(data) + packGPSDATA(gps_d);
     // send the data to the raspberry pi
     SerialRaspi.println(packed_data);
     // send the data to Telemetry 
     e32.sendMessage(packed_data);


     if(data.vel_bmp<SAFE_PARACHUTE_VEL){
      delay(1000);
     } 

     //check if altitute is within 10 m of base altitude , we start beeping buzzer
     if (data.bmpAltitude - groundAltitude <10) {
        pinMode(BuzzerPin,HIGH);
        delay(500);
        pinMode(BuzzerPin,LOW);
        delay(500);
     }
  }
  

}