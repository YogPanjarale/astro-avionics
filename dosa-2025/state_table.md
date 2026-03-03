# State Table and Flight Plan

## Mission requirements

1. flight computer powers, wait for 15 seconds for both boards to power on and boot , we beep the buzzer every one second
2. next 15 seconds establish connection between the boards, during first 30 seconds , we still beep every one second.
3. long beep after 30 seconds for 2 seconds.
4. get altitude reference , and calibrate IMU
    - we are doing oversampling , we only do one reading of bmp altitude.
    - for IMU we do 500 samples , and calibrate IMU.
5. continuity check should be done on ejection charges , every 300-400 loop cycles
6. When idle , arduino starts sending all sensor data to raspi , and arduino to telemetery , also keeps checking altitude from BMP and inegrating the IMU acceleration to see if we have entered flight mode
7. enter flight mode if altitude higher than 200m. , 
    in flight mode apogee is detected either when bmp velocity is 0 or
    - case 1 - IMU detects tip over
    - case 2- if IMU says tip over not detected
        - bmp and IMU velocity match at instant
        - when vel is small ( i.e 5m/s ) - deploy parachute
        - wait for vel to decrease if vel is high for parachute to deploy
        - WIP
    - keep the data in buffer 
8. deploy drouge after apogge is detected
9. deploy parachute when vel is small
10. enter descent mode after parachute deployed , keep sending sensor readings, turn on gps , send gps vel position data too.
11. if altitude is less than 20m  or velocity is <1m/s for 10 seconds (building or tree height) enter recovery mode
12. in recovery mode ,gps and keep sending location coordinates via telemetry.

## States

1. Boot
2. Connect - Establish Connection between arduino and raspi
3. Calibrate - calibrate IMU , get ref altitude from BMP
4. Idle - send data , watch for flight mode
5. Flight mode - keep track of imu vel and bmp vel and imu tipover to detect apogee, enter drouge state.
6. drouge state - deoply drouge, keep waiting for vel / altitute to decrease to enter parachute state
7. parachute state - deoply parachute , keep sending data, check for vel and altitude to enter recovery mode
8. recovery mode - keep sending location data via telemetry , and turn on buzzer , beep.

## Data

- all data is send to raspi while idle and descent(drouge/parachute) mode
- while flight mode , data is kept in buffer , send afterwards.
- recovery mode , only location, altitude is sent.
