#define GPS_RX 1  // GPS Module RX pin
#define GPS_TX 0  // GPS Module TX pin

#include <Arduino.h>

HardwareSerial GPS_Serial(3);  // Serial2 for GPS

struct GPSData {
    double latitude;
    char latDir;
    double longitude;
    char lonDir;
};

// Initialize GPS
void initGPS() {
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

// Convert NMEA format to decimal degrees
double convertToDecimalDegrees(const char *nmea, char direction) {
    double raw = atof(nmea);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimalDegrees = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') decimalDegrees *= -1;
    return decimalDegrees;
}

// Read and extract GPS data
GPSData readGPS() {
    GPSData data = {0.0, 'N', 0.0, 'E'};
    char buffer[100];
    int index = 0;
    bool reading = false;

    while (GPS_Serial.available()) {
        char c = GPS_Serial.read();
        Serial.print(c);  // Debugging raw data

        if (c == '$') { reading = true; index = 0; }
        if (reading) {
            buffer[index++] = c;
            if (c == '\n' || index >= 99) {
                buffer[index] = '\0';
                reading = false;

                if (strstr(buffer, "$GNGGA") || strstr(buffer, "$GNRMC")) {
                    char *token = strtok(buffer, ",");
                    int field = 0;

                    while (token != NULL) {
                        field++;
                        if (field == 3) data.latitude = convertToDecimalDegrees(token, 'N');
                        if (field == 4) data.latDir = token[0];
                        if (field == 5) data.longitude = convertToDecimalDegrees(token, 'E');
                        if (field == 6) data.lonDir = token[0];
                        token = strtok(NULL, ",");
                    }
                    break;
                }
            }
        }
    }
    return data;
}

void setup() {
    Serial.begin(115200);
    initGPS();
    Serial.println("L86 GPS Module Initialized...");
}

void loop() {
    GPSData reading = readGPS();

    if (reading.latitude != 0.0 && reading.longitude != 0.0) {
        Serial.print("Latitude: ");
        Serial.print(reading.latitude, 6);
        Serial.print(" ");
        Serial.println(reading.latDir);

        Serial.print("Longitude: ");
        Serial.print(reading.longitude, 6);
        Serial.print(" ");
        Serial.println(reading.lonDir);
    } else {
        Serial.println("Waiting for GPS fix...");
    }

    delay(1000);
}