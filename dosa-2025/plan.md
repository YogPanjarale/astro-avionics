# Development Plan - Astro Avionics Flight Computer

This document contains the original development plan and implementation details for the flight computer system.

## Arduino Progress

- [x] Make individual .cpp and .h files for each sensor and task
- [x] Include them in main and finalize the logic
- [x] Debugging and testing

## Raspi Progress

- [x] Finalize the logic and implement it in Python
- [x] Debugging and testing

## Flight Computer Code Plan

1. [x] Write out the cases and conditions of what happens when
2. [x] Write the possible states and cases for transition between states & draw diagram of state machine
3. [x] Write test code for individual components (i.e. taking readings from GPS, BMP390, IMU, communication with E32, Raspi, triggering relay, triggering buzzer, testing ejection charges)
4. [x] Test the code
5. [x] Back to step 3 until code works
6. [x] Implement the state machine using state tables with placeholder function blocks for individual components
7. [x] Integrate the test code for individual components from step 3 to implement the placeholder functions
8. [x] Test the code with everything

## How to Run

### Arduino (PlatformIO)

1. **Install PlatformIO Extension**
   - Open VS Code
   - Go to Extensions (Ctrl+Shift+X)
   - Search for "PlatformIO IDE"
   - Install and restart VS Code

2. **Open Project**
   - Click on PlatformIO icon in the sidebar (ant icon)
   - Click "Open Project" or "Pick a folder"
   - Navigate to and select the `arduino` folder
   - PlatformIO will automatically detect the project configuration

3. **Build Project**
   - Click the checkmark (✓) icon in the bottom toolbar, or
   - Use shortcut: `Ctrl+Alt+B` (Windows/Linux) or `Cmd+Alt+B` (Mac)
   - Wait for compilation to complete

4. **Upload to Arduino**
   - Connect Arduino Nano ESP32 via USB
   - Click the arrow (→) icon in the bottom toolbar, or
   - Use shortcut: `Ctrl+Alt+U` (Windows/Linux) or `Cmd+Alt+U` (Mac)
   - Wait for upload to complete

5. **Monitor Serial Output**
   - Click the plug icon (🔌) in the bottom toolbar, or
   - Use shortcut: `Ctrl+Alt+S` (Windows/Linux) or `Cmd+Alt+S` (Mac)
   - Set baud rate to 115200 if needed

### Raspberry Pi

1. **Install Dependencies**
   ```bash
   pip3 install aioserial RPi.GPIO adafruit-circuitpython-bmp3xx adafruit-blinka
   ```

2. **Enable Hardware Interfaces**
   ```bash
   sudo raspi-config
   ```
   - Navigate to: Interface Options → I2C → Enable
   - Navigate to: Interface Options → Serial Port → Enable
   - Reboot if prompted

3. **Verify BMP390 Connection**
   ```bash
   sudo apt-get install i2c-tools
   i2cdetect -y 1
   ```
   - Should show `0x77` in the output

4. **Configure Sea Level Pressure**
   - Edit `Raspi/main.py`
   - Find line: `bmp.sea_level_pressure = 1015`
   - Set to your local sea level pressure in hPa

5. **Run the Flight Controller**
   ```bash
   cd Raspi
   python3 main.py
   ```

6. **Run Camera Recording (Optional)**
   ```bash
   # Start recording
   ./start_recording.sh
   
   # Stop recording
   ./stop_recording.sh
   ```

## Flight Regimes (State Machine Implementation)

The flight computer uses a state machine with an enum and switch-case structure in the main loop.

### State Enum Definition

```cpp
enum State {
  BOOT,        // Initial boot sequence
  CONN,        // Connection establishment
  CALIB,       // Sensor calibration
  IDLE,        // Pre-flight monitoring
  FLIGHT,      // Active flight mode
  DROGUE,      // Drogue deployment
  PARACHUTE,   // Main parachute deployment
  RECOVERY,    // Post-landing recovery
};

State state = BOOT;  // Global state variable
```

### State Machine Loop Structure

```cpp
void loop() {
  while(state == BOOT) {
    // Boot sequence code
    // Beep buzzer every second for 15 seconds
    setState(CONN);
  }
  
  while(state == CONN) {
    // Connection establishment code
    // Establish communication with Raspi and E32
    setState(CALIB);
  }
  
  while(state == CALIB) {
    // Calibration code
    // Calibrate IMU, set ground altitude
    setState(IDLE);
  }
  
  while(state == IDLE) {
    // Pre-flight monitoring
    // Send sensor data, detect launch (>200m)
    if (altitude > 200) setState(FLIGHT);
  }
  
  while(state == FLIGHT) {
    // Flight mode
    // Monitor altitude/velocity, detect apogee
    if (apogee_detected) setState(DROGUE);
  }
  
  while(state == DROGUE) {
    // Drogue deployment
    // Deploy drogue, wait for confirmation
    setState(PARACHUTE);
  }
  
  while(state == PARACHUTE) {
    // Main parachute deployment
    // Deploy main when safe velocity/altitude
    setState(RECOVERY);
  }
  
  while(state == RECOVERY) {
    // Recovery mode
    // GPS tracking, buzzer activation
  }
}
```

### State Transition Function

The `setState()` function updates both the enum state variable and the status register bits:

```cpp
void setState(State s) {
  // Update status register bits 11, 12, 13
  switch (s) {
    case BOOT:      // 000
      status &= ~(S10_h | S11h | S12_h);
      break;
    case CONN:      // 001
      status &= ~(S10_h | S11h);
      status |= S12_h;
      break;
    case CALIB:     // 010
      status &= ~(S10_h | S12_h);
      status |= S11h;
      break;
    case IDLE:      // 011
      status &= ~(S10_h);
      status |= S11h | S12_h;
      break;
    case FLIGHT:    // 100
      status &= ~(S11h | S12_h);
      status |= S10_h;
      break;
    case DROGUE:    // 101
      status &= ~(S10_h | S11h);
      status |= S12_h;
      break;
    case PARACHUTE: // 110
      status &= ~(S10_h);
      status |= S11h | S12_h;
      break;
    case RECOVERY:  // 111
      status |= S10_h | S11h | S12_h;
      break;
  }
  state = s;  // Update enum state
}
```

## Sensor States (Status Register)

The system uses a 16-bit status register (`uint16_t status`) to track sensor health and system state using bit manipulation.

### Status Register Bit Definitions

```cpp
// Sensor status bits (0-4)
#define RPI_h    (1 << 0)  // Raspberry Pi connection status
#define GPS_h    (1 << 1)  // GPS module status
#define LORA_h   (1 << 2)  // LoRa (E32) module status
#define BMP_h    (1 << 3)  // BMP390 sensor status
#define IMU_h    (1 << 4)  // IMU sensor status

// Ejection charge continuity bits (5-7)
#define ECd_h    (1 << 5)  // Drogue ejection charge continuity
#define ECm_h    (1 << 6)  // Main ejection charge continuity
#define ECb_h    (1 << 7)  // Backup ejection charge continuity

// Ejection charge deployment status bits (8-10)
#define ECrd_h   (1 << 8)  // Drogue ejection charge deployed
#define ECrm_h   (1 << 9)  // Main ejection charge deployed
#define ECrb_h   (1 << 10) // Backup ejection charge deployed

// Flight state bits (11-13)
#define S10_h    (1 << 11) // State bit 0
#define S11h     (1 << 12) // State bit 1
#define S12_h    (1 << 13) // State bit 2

// Special status bits (14-15)
#define TIP_OVER      (1 << 14) // Tip-over detected
#define DROGUE_DEPLOYED (1 << 15) // Drogue deployment confirmed
```

### Status Register Bit Layout

```
Bit:  15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
      ─── ─── ─── ─── ─── ─── ─── ─── ─── ─── ─── ─── ─── ─── ───
      │   │   │   │   │   │   │   │   │   │   │   │   │   │   │   │
      │   │   │   └───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴─── Sensor status
      │   │   │       │   │   │   │   │   │   │   │   │   │   │   │
      │   │   │       │   │   │   │   │   │   │   │   │   │   │   └── RPI
      │   │   │       │   │   │   │   │   │   │   │   │   │   └────── GPS
      │   │   │       │   │   │   │   │   │   │   │   │   └───────── LORA
      │   │   │       │   │   │   │   │   │   │   │   └───────────── BMP
      │   │   │       │   │   │   │   │   │   │   └───────────────── IMU
      │   │   │       │   │   │   │   │   │   └────────────────────── ECd (continuity)
      │   │   │       │   │   │   │   │   └────────────────────────── ECm (continuity)
      │   │   │       │   │   │   │   └────────────────────────────── ECb (continuity)
      │   │   │       │   │   │   └────────────────────────────────── ECrd (deployed)
      │   │   │       │   │   └────────────────────────────────────── ECrm (deployed)
      │   │   │       │   └────────────────────────────────────────── ECrb (deployed)
      │   │   └───────┴────────────────────────────────────────────── State (bits 11-13)
      │   └─────────────────────────────────────────────────────────── TIP_OVER
      └─────────────────────────────────────────────────────────────── DROGUE_DEPLOYED
```

### Status Register Manipulation

```cpp
// Set a status bit to true
status |= RPI_h;          // Set Raspberry Pi status to connected

// Set a status bit to false
status &= ~LORA_h;        // Set LoRa status to disconnected

// Toggle a status bit
status ^= BMP_h;          // Toggle BMP status

// Check if a status bit is set
if ((status & IMU_h) != 0) {
  // IMU is alive
}

// Check multiple bits
if ((status & BMP_h) && (status & IMU_h)) {
  // Both BMP and IMU are alive
}

// Clear multiple bits
status &= ~(S10_h | S11h | S12_h);  // Clear all state bits

// Set multiple bits
status |= S11h | S12_h;  // Set state bits for IDLE (011)
```

### Example Usage in Code

```cpp
// During sensor initialization
if (bmpSensor.begin()) {
  status |= BMP_h;  // Mark BMP as alive
} else {
  status &= ~BMP_h; // Mark BMP as dead
  sendAllSerial("BMP Sensor not working");
}

// Check sensor before using
if (status & BMP_h) {
  float altitude = bmpSensor.getAltitude();
} else {
  // Use fallback or skip reading
}

// Update ejection charge continuity
if (checkMainEjectionCharges()) {
  status |= ECm_h;  // Main charge has continuity
} else {
  status &= ~ECm_h; // Main charge has no continuity
}

// After deploying drogue
if (deployment_successful) {
  status |= ECrd_h;      // Mark as deployed
  status &= ~ECd_h;      // Clear continuity (charge is fired)
  status |= DROGUE_DEPLOYED; // Set deployment flag
}
```

## Individual Components and Functions

### Implemented Components

1. [x] GPS Readings (`gps.cpp`, `gps.h`)
   - `getCoordinates()` - Get GPS coordinates
   - `readGPSData()` - Read complete GPS data structure
   - `packGPSDATA()` - Pack GPS data for transmission
   - `checkAlive()` - Check GPS module status

2. [x] BMP390 Reading (`bmp.cpp`, `bmp.h`)
   - `getAltitude()` - Get altitude from pressure
   - `getPressure()` - Get raw pressure reading
   - `getVelocity()` - Calculate velocity from altitude changes
   - `begin()` - Initialize sensor, returns true if successful

3. [x] IMU Reading (`imu.cpp`, `imu.h`)
   - `getRollPitchYaw()` - Get orientation data
   - `getAcceleration()` - Get acceleration vector
   - `getHeightIMU()` - Get altitude from integrated acceleration
   - `getVelocityIMU()` - Get velocity from integrated acceleration
   - `isRocketTippingOver()` - Detect tip-over condition
   - `setupIMU()` - Initialize sensor
   - `calibrateIMU()` - Calibrate with N samples
   - `readIMU()` - Read IMU data structure

4. [x] E32 Communication (`E32.cpp`, `E32.h`)
   - `sendMessage()` - Send data via LoRa
   - `setup()` - Initialize E32 module
   - `checkAlive()` - Verify module is responding

5. [x] Raspi Communication (in `main.cpp`)
   - `SerialRaspi.println()` - Send data to Raspberry Pi
   - `SerialRaspi.readStringUntil()` - Read data from Raspberry Pi
   - Ping/Pong handshake for connection verification

6. [x] Continuity Check (`checkEjectionCharges.cpp`, `checkEjectionCharges.h`)
   - `checkMainEjectionCharges()` - Check main charge continuity
   - `checkDrogueEjectionCharges()` - Check drogue charge continuity
   - `checkBackupEjectionCharges()` - Check backup charge continuity
   - `checkAllEjectionChargeContinuity()` - Check all charges (called every 200 cycles)

7. [x] Trigger Ejection Charges (`triggerEjectionCharges.cpp`, `triggerEjectionCharges.h`)
   - `triggerMainEjectionCharges(duration)` - Fire main charge
   - `triggerDrogueEjectionCharges(duration)` - Fire drogue charge
   - `triggerBackupEjectionCharges(duration)` - Fire backup charge

8. [x] Buzzer Control (`buzzer.cpp`, `buzzer.h`)
   - `playCalibrationStartTone()` - Play calibration tone
   - Direct pin control for beeping patterns

9. [x] Raspberry Pi Functions (`Raspi/main.py`)
   - `readBytes()` - Read data from Arduino
   - `sendBytes()` - Send data to Arduino
   - `getAltitude()` - Get altitude from BMP390
   - `triggerEjectionCharges()` - Trigger charges via GPIO
   - `storeDataToSDCard()` - Log flight data
   - `storeCameraFootage()` - Manage camera recording

## Data Packing and Transmission

### Arduino Data Structure

```cpp
struct Data {
  unsigned long time;      // Timestamp in milliseconds
  float bmpAltitude;       // Altitude from BMP390 (m)
  float imuAltitude;       // Altitude from IMU integration (m)
  float pressure;          // Pressure reading (Pa)
  float accel_x;           // X-axis acceleration (m/s²)
  float accel_y;           // Y-axis acceleration (m/s²)
  float accel_z;           // Z-axis acceleration (m/s²)
  float vel_bmp;           // Velocity from BMP (m/s)
  float vel_imu;           // Velocity from IMU (m/s)
  uint16_t statusReg;      // Status register
};
```

### Data Packing Function

```cpp
String packDATA(Data data) {
  // Convert status register to binary string
  String binaryStatus = "";
  for (int i = 15; i >= 0; i--) {
    binaryStatus += String((data.statusReg >> i) & 1);
  }
  
  // Pack as comma-separated values
  return String(data.bmpAltitude) + "," + 
         String(data.imuAltitude) + "," + 
         String(data.pressure) + "," + 
         String(data.accel_x) + "," + 
         String(data.accel_y) + "," + 
         String(data.accel_z) + "," + 
         String(data.vel_bmp) + "," + 
         String(data.vel_imu) + "," + 
         binaryStatus;
}
```

### Transmission

- **To Raspberry Pi**: Via Serial2 (115200 baud) - All sensor data and logs
- **To Telemetry (E32)**: Via Serial1 (115200 baud) - Real-time flight data
- **To USB Serial**: Via Serial (115200 baud) - Debug output

Maximum E32 payload: 56 bytes at 9600 baud (currently sending ~42 bytes)

## Code Structure

### Individual Components Test Code
- Location: `ComponentTests/`
- Format: Arduino IDE `.ino` files
- Purpose: Test individual sensors and components before integration

### State Machine Implementation
- Location: `arduino/src/main.cpp`
- Format: PlatformIO C++ project
- Framework: Arduino framework on ESP32

### Raspberry Pi Code
- Location: `Raspi/main.py`
- Format: Python 3 with asyncio
- Dependencies: aioserial, RPi.GPIO, adafruit-circuitpython-bmp3xx

## Testing and Validation

### Component Testing
- [x] GPS module reading coordinates
- [x] BMP390 altitude and pressure readings
- [x] IMU acceleration and orientation
- [x] E32 LoRa communication
- [x] Raspberry Pi serial communication
- [x] Relay triggering for ejection charges
- [x] Buzzer control
- [x] Continuity checking circuits

### Integration Testing
- [x] State machine transitions
- [x] Sensor data collection and transmission
- [x] Apogee detection algorithms
- [x] Ejection charge deployment sequence
- [x] Data logging to SD card
- [x] Telemetry transmission

### Flight Testing
- [x] Ground testing with all components
- [x] Simulated flight scenarios
- [x] End-to-end system validation

## Notes

- Continuity checking is performed every 200 loop cycles (approximately every 10 seconds at 20Hz loop rate)
- IMU calibration uses 500 samples at 5ms intervals (total 2.5 seconds)
- Ground altitude reference is set during calibration phase
- Flight mode is triggered when altitude exceeds 200m above ground
- Apogee detection uses multiple methods: BMP velocity, IMU velocity, and tip-over detection
- Main parachute deployment occurs when altitude < 500m or velocity < -30 m/s
- Backup parachute triggers if velocity < -45 m/s in recovery mode
