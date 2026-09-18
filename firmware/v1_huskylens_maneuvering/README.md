# Version 1 HuskyLens manoeuvring firmware

`v1_huskylens_maneuvering.ino` is the Version 1 ESP32 program supplied by the team for obstacle-course manoeuvring.

## Intended hardware

- ESP32
- HuskyLens using colour-recognition mode
- BNO055 IMU
- Front, left and right HC-SR04 ultrasonic sensors
- Steering servo
- Brushed DC motor and compatible driver
- Start button on GPIO 0
- Bluetooth serial for telemetry

## Behaviour covered by this version

The program detects coloured pillars through HuskyLens, uses ultrasonic measurements for wall and clearance checks, stabilises heading from the IMU, and controls steering and rear drive through a state-based manoeuvring flow. It also contains a parking-exit sequence, turn handling and recovery behaviour.

## Before uploading

1. Confirm every GPIO assignment against the assembled Version 1 vehicle.
2. Verify the motor-driver, servo and sensor supply voltages before applying power.
3. Confirm the HuskyLens colour IDs match its trained model.
4. Test with wheels raised before running on the course.
5. Record the tested revision and calibration values in this repository's run log and calibration folders.
