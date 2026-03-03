#include <SoftwareSerial.h>

// The rpi will wait for 20 seconds before entering failure mode, make sure some data is being sent every 10 seconds
// Arduino does not care if raspi is alive or dead
// Implemented stop recording and exit functions on raspi (STOPREC, EXIT)