#include "sdlogger.h"

SDLogger logger;

bool SDLogger::begin()
{
    // Configure SPI pins for Pico
    SPI.setRX(SD_MISO);
    SPI.setTX(SD_MOSI);
    SPI.setSCK(SD_SCK);

    if (!SD.begin(SD_CS, SPI))
        return false;

    logFile = SD.open("flight.csv", FILE_WRITE);
    if (!logFile)
        return false;

    logFile.println("time,altitude,velocity,state");

    return true;
}

void SDLogger::append(const char* data)
{
    int len = strlen(data);

    if (len >= BUFFER_SIZE)
        return;

    if (activeIndex + len >= BUFFER_SIZE)
    {
        if (flushBuffer == nullptr)
        {
            flushBuffer = activeBuffer;
            flushSize = activeIndex;

            activeBuffer = (activeBuffer == bufferA) ? bufferB : bufferA;
            activeIndex = 0;
        }
        else
        {
            return;
        }
    }

    memcpy(&activeBuffer[activeIndex], data, len);
    activeIndex += len;
}

void SDLogger::logSample()
{
    char line[96];

    unsigned long t = millis();
    int state = static_cast<int>(flightState);

    float alt = altitude;
    float vel = velocity;

    snprintf(line, sizeof(line),
             "%lu,%.3f,%.3f,%d\n",
             t,
             alt,
             vel,
             state);

    append(line);
}

void SDLogger::logString(const char* msg)
{
    char line[128];

    snprintf(line, sizeof(line),
             "%lu,%s\n",
             millis(),
             msg);

    append(line);
}

void SDLogger::logContinuity(bool mcont, bool bcont)
{
    char line[128];

    const char* mainStatus   = mcont ? "TRUE" : "FALSE";
    const char* backupStatus = bcont ? "TRUE" : "FALSE";

    snprintf(line, sizeof(line),
             "%lu,Continuity => Main: %s, Backup: %s\n",
             millis(),
             mainStatus,
             backupStatus);

    append(line);
}

void SDLogger::flush()
{
    if (flushBuffer == nullptr)
        return;

    logFile.write((uint8_t*)flushBuffer, flushSize);
    logFile.flush();

    flushBuffer = nullptr;
}