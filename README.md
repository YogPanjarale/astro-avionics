# Astro Avionics - Flight Computer

Flight computer system for Astro Club BITS Pilani's rocket missions. This system provides autonomous flight control, telemetry, and recovery capabilities.

## Overview

The flight computer consists of two main components:
- **Arduino Nano ESP32**: Primary flight controller handling sensors, state machine, and ejection charge control
- **Raspberry Pi**: Secondary controller for data logging, camera control, and backup ejection charge triggering

## Features

- **Multi-sensor Integration**: GPS, BMP390 barometric pressure sensor, ICM20X IMU
- **State Machine**: Autonomous flight regime management (Boot → Connect → Calibrate → Idle → Flight → Drogue → Parachute → Recovery)
- **Ejection Charge Control**: Automated deployment of drogue and main parachutes with continuity checking
- **Telemetry**: Real-time data transmission via E32 LoRa module
- **Data Logging**: Comprehensive flight data logging to Raspberry Pi SD card
- **Recovery System**: GPS tracking and buzzer activation for recovery

## Project Structure

```
astro-avionics/
├── arduino/              # Arduino flight controller code
│   ├── src/              # Source files
│   │   ├── main.cpp      # Main state machine implementation
│   │   ├── bmp.*         # BMP390 sensor interface
│   │   ├── gps.*         # GPS module interface
│   │   ├── imu.*         # IMU sensor interface
│   │   ├── E32.*         # LoRa telemetry module
│   │   ├── buzzer.*      # Buzzer control
│   │   ├── checkEjectionCharges.*    # Continuity checking
│   │   └── triggerEjectionCharges.* # Ejection charge control
│   ├── platformio.ini    # PlatformIO configuration
│   └── test/             # Component test files
├── Raspi/                # Raspberry Pi code
│   ├── main.py           # Main Python flight controller
│   ├── start_recording.sh # Camera recording start script
│   └── stop_recording.sh  # Camera recording stop script
├── ComponentTests/       # Individual component test sketches
├── state_table.md        # Flight state machine documentation
└── Notes.md              # Hardware connection details
```

## Hardware Requirements

### Arduino Nano ESP32
- BMP390 Barometric Pressure Sensor (I2C)
- ICM20X IMU (I2C)
- GPS Module (UART)
- E32 LoRa Module (UART)
- Buzzer
- Relay modules for ejection charges
- Continuity check circuits

### Raspberry Pi
- BMP390 Barometric Pressure Sensor (I2C)
- Camera module
- GPIO-controlled relays
- Status LEDs

## Hardware Connections

### Arduino Nano ESP32 Pinout

| Component | Pin | Description |
|-----------|-----|-------------|
| GPS TX | D0 | GPS transmit |
| GPS RX | D1 | GPS receive |
| Main Relay | D9 | Main parachute ejection |
| Drogue Relay | D8 | Drogue parachute ejection |
| Backup Relay | D7 | Backup parachute ejection |
| Main Continuity Power | D5 | Main continuity check power |
| Main Continuity Detect | A0 | Main continuity detection |
| Drogue Continuity Power | D4 | Drogue continuity check power |
| Drogue Continuity Detect | D6 | Drogue continuity detection |
| Backup Continuity Power | D3 | Backup continuity check power |
| Backup Continuity Detect | D2 | Backup continuity detection |
| BMP390 SDA | A4 | I2C data line |
| BMP390 SCL | A5 | I2C clock line |
| BMP390 Power | D10 | BMP390 power control |
| Buzzer | A1 | Buzzer control |
| E32 M0 | D11 | E32 mode control |
| E32 M1 | D12 | E32 mode control |
| E32 RX | A3 | E32 receive |
| E32 TX | A2 | E32 transmit |
| Raspi RX | A6 | Raspberry Pi receive |
| Raspi TX | A7 | Raspberry Pi transmit |

### Raspberry Pi Pinout

| Component | GPIO | Description |
|-----------|------|-------------|
| GPS Force On | 13 | GPS power control |
| Backup Relay | 22 | Backup parachute ejection |
| Drogue Relay | 27 | Drogue parachute ejection |
| Main Relay | 17 | Main parachute ejection |
| Status LED | 25 | Status indicator |
| Telemetry LED | 24 | Telemetry indicator |
| BMP390 SDA | GPIO2 | I2C data line |
| BMP390 SCL | GPIO3 | I2C clock line |

For detailed connection information, see `Notes.md`.

## Flight States

The flight computer operates through the following states:

1. **BOOT**: Initial power-on sequence (15 seconds), buzzer beeps every second
2. **CONN**: Connection establishment between Arduino and Raspberry Pi (15 seconds)
3. **CALIB**: Sensor calibration - IMU calibration (500 samples) and ground altitude reference
4. **IDLE**: Pre-flight monitoring, sending sensor data, waiting for launch detection (>200m altitude)
5. **FLIGHT**: Active flight mode, monitoring altitude and velocity, detecting apogee
6. **DROGUE**: Apogee detected, deploying drogue parachute
7. **PARACHUTE**: Deploying main parachute when safe velocity/altitude reached
8. **RECOVERY**: Post-landing mode, GPS tracking, buzzer activation for recovery

For detailed state machine documentation, see `state_table.md`.

## Setup Instructions

### Arduino Setup

1. **Install PlatformIO**
   - Install PlatformIO extension in VS Code
   - Restart VS Code

2. **Open Project**
   - Open the `arduino` folder in PlatformIO
   - PlatformIO will automatically detect the project configuration

3. **Build and Upload**
   - Click the build icon (✓) to compile
   - Connect Arduino Nano ESP32 via USB
   - Click the upload icon (→) to flash the firmware

### Raspberry Pi Setup

1. **Install Dependencies**
   ```bash
   pip install aioserial RPi.GPIO adafruit-circuitpython-bmp3xx adafruit-blinka
   ```

2. **Enable Hardware Interfaces**
   - Enable I2C: `sudo raspi-config` → Interface Options → I2C → Enable
   - Enable Serial: `sudo raspi-config` → Interface Options → Serial Port → Enable
   - Verify BMP390 address: `i2cdetect -y 1` (should show 0x77)

3. **Configure Sea Level Pressure**
   - Edit `Raspi/main.py` and set `bmp.sea_level_pressure` to your local sea level pressure (in hPa)

4. **Run Flight Controller**
   ```bash
   cd Raspi
   python3 main.py
   ```

## Usage

### Pre-Flight Checklist

1. Verify all hardware connections
2. Check ejection charge continuity (system will verify during calibration)
3. Set correct sea level pressure in Raspberry Pi code
4. Ensure GPS has clear sky view for initial fix
5. Verify telemetry link (E32 module)

### Flight Sequence

1. Power on both Arduino and Raspberry Pi
2. System enters BOOT state (15 seconds of beeping)
3. Connection establishment (15 seconds)
4. Calibration phase:
   - IMU calibration (500 samples, ~2.5 seconds)
   - Ground altitude reference set
   - Ejection charge continuity check
5. System enters IDLE state, monitoring sensors
6. Launch detection: System enters FLIGHT mode when altitude > 200m
7. Apogee detection: Multiple methods (BMP velocity, IMU velocity, tip-over detection)
8. Drogue deployment: Automatic at apogee
9. Main parachute deployment: When altitude < 500m or velocity < -30 m/s
10. Recovery mode: GPS tracking and buzzer activation when near ground

### Data Logging

Flight data is automatically logged to:
- Raspberry Pi: `~/flightlogs/flight_HHMMSS.log`
- BMP390 data: `~/flightlogs/bmp_HHMMSS.log`

## Telemetry

The E32 LoRa module transmits flight data in real-time. Data format includes:
- BMP altitude and velocity
- IMU altitude and velocity
- Pressure readings
- Acceleration data (x, y, z)
- Status register (sensor health, ejection charge status, flight state)

## Troubleshooting

### Arduino Issues
- **Sensors not detected**: Check I2C connections and power
- **Serial communication fails**: Verify baud rates and pin connections
- **Ejection charges not firing**: Check continuity status and relay connections

### Raspberry Pi Issues
- **BMP390 not found**: Verify I2C is enabled and address is correct (0x77)
- **Serial communication fails**: Check serial port configuration (`/dev/ttyS0`)
- **Camera not recording**: Verify camera module is connected and enabled

## Safety Notes

⚠️ **IMPORTANT**: This system controls pyrotechnic ejection charges. Always:
- Verify continuity before flight
- Follow proper safety procedures
- Test ejection charges in safe, controlled environment
- Have backup recovery systems in place
- Never test with live charges in populated areas

## Documentation

- `plan.md`: Development plan, state variable usage, and technical implementation details
- `state_table.md`: Detailed flight state machine and mission requirements
- `Notes.md`: Hardware connection details and technical notes
- `ComponentTests/`: Individual component test sketches for debugging

## License

This project is developed for Astro Club BITS Pilani.
