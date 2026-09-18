# Version 2 Open Round ToF firmware

`v2_open_round_tof.ino` is the Version 2 ESP32-C3 program supplied by the team for Open Round manoeuvring with time-of-flight distance sensing.

## Intended hardware

- ESP32-C3
- Three VL53L0X ToF sensors through a TCA9548A I2C multiplexer
- BNO055 IMU through the same I2C multiplexer
- Steering servo
- Brushed DC motor and compatible driver
- NeoPixel status strip
- Start button

## Sensor routing in this version

| Device | TCA9548A channel |
| --- | --- |
| Front ToF | 0 |
| Right ToF | 1 |
| Left ToF | 2 |
| BNO055 IMU | 4 |

The code uses I2C pins GPIO 8 and GPIO 9, a start button on GPIO 10, steering servo on GPIO 6, NeoPixel data on GPIO 7, and motor control on GPIO 0 and GPIO 1. Verify the board, driver and wiring before upload.

## Behaviour covered by this version

The program initialises the three ToF channels and IMU, locks the initial heading at the start button, centres between walls, detects corners, completes direction-locked turns, and stops after the configured twelve-turn sequence.

## Before uploading

1. Verify TCA9548A channel assignments and every physical I2C connection.
2. Confirm the motor driver accepts the configured PWM and direction signals.
3. Check servo supply voltage and steering end stops with the wheels raised.
4. Confirm ToF readings and BNO055 heading before pressing the start button.
5. Save the tested revision, settings and run outcome in the calibration and run-log folders.
