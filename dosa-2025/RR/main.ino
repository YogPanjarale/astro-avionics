#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#define LOGDIR "/logs"
#define ALTLOG "/logs/altlog.txt"
#define LOGFILE "/logs/log.txt"
#define TIMEOUT 20000 // These are in millis
#define MAIN_VEL -30  // Negative velocity !!!
#define BACKUP_VEL -40
#define seaLevelPressure 101600
#define chipSelect 5
#define DROGUE A6
#define MAIN A7
#define BACKUP A3
#define STATUS 7
#define TEL 6
#define ARX A1
#define ATX A0

SoftwareSerial Serial1(ARX, ATX);
enum State {LOGGING, GROUND, FLIGHT, DESCENT, LANDING, SHUTDOWN};
State mode = LOGGING;
Adafruit_BMP3XX bmp;
float prevAltitude = 0;
unsigned long prevTime = 0;

File logFile;
File alt;

float getAltitude(){
  float pressure = bmp.pressure;       // Pressure in Pascals
  float temperature = bmp.temperature; // Temperature in Celsius
  float altitude = 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
  return altitude;
}

float getVelocity(){
  static unsigned long prevTime = 0;                   // Previous time
  static float prevVelocity = 0;                       // Previous velocity

  if (millis() - prevTime < 45){// Update velocity every 45ms
    return prevVelocity;
  }
  unsigned long currentTime = millis();                // Get current time
  float currentAltitude = getAltitude();               // Get current altitude
  float deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds
  float velocity = 0;                                  // Default velocity
  if (deltaTime > 0){ // Avoid division by zero
    velocity = (currentAltitude - prevAltitude) / deltaTime;
  }

  // Update previous values
  prevAltitude = currentAltitude;
  prevTime = currentTime;
  prevVelocity = velocity;
  return velocity;
}

void triggerCharge(int pin){
  digitalWrite(pin, HIGH);
  delay(2000);
  digitalWrite(pin, LOW);
}

void writeLog(String data){
  static uint8_t logcounter = 0;
  logFile.println(data);
  if(++logcounter > 3)
    logFile.flush();
}

void altLog(float data){
  static uint8_t altcounter = 0;
  char str[10];
  dtostrf(data, 9, 3, str);
  alt.println(str);
  if(++altcounter > 10)
    alt.flush();
}

void statusLED(){
  switch(mode){
    case LOGGING:
      digitalWrite(STATUS, HIGH);
      digitalWrite(TEL, HIGH);
      break;
    case SHUTDOWN:
      digitalWrite(STATUS, LOW);
      digitalWrite(TEL, LOW);
      break;
    default:
      digitalWrite(STATUS, HIGH);
      digitalWrite(TEL, LOW);
  }
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);

  // BMP init
  Wire.begin();
  if (!bmp.begin_I2C(119)){
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X); // new value should be updated every 40ms (25Hz)
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);  // new value should be updated every 40ms (25Hz)

  // SD init
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)){
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done.");
  if(!SD.exists(LOGDIR)){
    Serial.println("Log dir doesnt exist. Creating it.");
    SD.mkdir(LOGDIR);
  }
  else
    Serial.println("Log dir already exists.");
  
  logFile = SD.open(LOGFILE, FILE_WRITE); // Open file and keep flushing data to the log
  alt = SD.open(ALTLOG, FILE_WRITE);

  // Pin init
  pinMode(MAIN, OUTPUT);
  pinMode(DROGUE, OUTPUT);
  pinMode(BACKUP, OUTPUT);
  pinMode(STATUS, OUTPUT);
  pinMode(TEL, OUTPUT);
}

void loop(){
  static float groundAltitude = getAltitude();
  static uint8_t counter = 0;
  String grnd = "Ground altitude:" + String(groundAltitude, 2);
  writeLog(grnd);
  switch(mode){
    case LOGGING:
      statusLED();
      unsigned long int last_packet_time = millis();
      String data = "";
      bool arduino_status = 1;
      while(arduino_status){
        if(Serial1.available()){
          data = Serial1.readStringUntil('\n');
          if(data != ""){
            data.trim();
            writeLog(data);
            last_packet_time = millis();
            switch(data){
              case "EXIT":
              mode = SHUTDOWN;
              arduino_status = 0;
              break;
              case "PING":
              Serial1.println("PONG");
              break;
            }
          }
        }
        if(millis() - last_packet_time > TIMEOUT){
          mode = GROUND;
          statusLED();
          writeLog("Arduino timed out entering failure state!");
          arduino_status = 0;
        }
      }
      break;
    case GROUND:
      if(getAltitude() - groundAltitude > 200){
        mode = FLIGHT;
        writeLog("Entering flight mode!");
      }
      altLog(getAltitude());
      delay(45);
      break;
    case FLIGHT:
      if(getVelocity() < 0){
        counter ++;
      }
      else
        counter = 0;
      if(counter > 6){
        counter = 0;
        writeLog("Apogee detected!");
        triggerCharge(DROGUE);
        mode = DESCENT;
      }
      altLog(getAltitude());
      delay(45);
      break;
    case DESCENT:
      if(getAltitude() - groundAltitude < 500 || getVelocity() < MAIN_VEL){
        writeLog("Deploying main!");
        triggerCharge(MAIN);
      }
      altLog(getAltitude());
      delay(45);
      break;
    case LANDING:
      if(getVelocity() < BACKUP_VEL){
        writeLog("Deploying backup!");
        triggerCharge(BACKUP);
      }
      if(abs(getVelocity()) < 2.5)
        counter ++;
      else
        counter = 0;
      if(counter > 10){
        counter = 0;
        writeLog("Detected landing!");
        mode = SHUTDOWN;
      }
      altLog(getAltitude());
      delay(45);
      break;
    case SHUTDOWN:
      if(!counter){
        alt.flush();
        logFile.flush();
        alt.close();
        logFile.close();
        counter ++;
      }
      delay(2000);
      break;
  }
}
