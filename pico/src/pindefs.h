#ifndef PINDEF
#define PINDEF

// I2C connection for I2C rail B  MCU and sensors
// I2C1
#define SDA_B 20
#define SCL_B 21

// I2C connection for I2C rail A  MCU and sensors
// I2C0
#define SDA_A 10
#define SCL_A 11

// SPI connection for SD card and LoRa
#define SPI_SCK 2
#define SPI_MOSI 3
#define SPI_MISO 4
#define SPI_CS_LORA 5
#define LORA_RESET 6
#define LORA_DIO0 7
#define SPI_CS_SDCARD 26

// Pyro channel connections
#define MAIN_SENSE 13 // main pyro channel sense
#define MAIN_TRIG 14 // main pyro channel trigger
#define MAIN_CONT 15 // main pyro channel continuity check
#define BACKUP_SENSE 17 // backup pyro channel sense
#define BACKUP_TRIG 16 // backup pyro channel trigger
#define BACKUP_CONT 22 // backup pyro channel continuity check

// GPS connection
// WARNING: MAYBE Reverse these pins if GPS does not work.
#define GPS_TX 8   
#define GPS_RX 9


#endif