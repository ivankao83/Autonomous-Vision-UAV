
# ESP32-S3 Drone Sensor & Telemetry Firmware

## Overview

This directory contains the ESP32-S3 firmware for a custom autonomous tailsitter UAV with thrust vectoring.

The firmware is responsible for sensor initialization, real time data acquisition, attitude estimation, signal filtering, and telemetry visualization through a TFT display.

The drone is designed around two processing systems:

- **ESP32-S3:** Low-level sensor integration, hardware monitoring, and embedded control.
- **Raspberry Pi:** Computer vision, object recognition, and autonomous target following (planned integration).

> **Development Status:** Sensor integration and telemetry development. Flight stabilization and autonomous control are not yet implemented in this firmware.

---

## Hardware Components

| Component | Function | Interface |
|-----------|----------|-----------|
| ESP32-S3 | Main microcontroller | — |
| BMI160 | Accelerometer & gyroscope | I2C |
| QMC5883P | Magnetometer / compass | I2C |
| BMP280 | Pressure, temperature & altitude | I2C |
| PMW3901 | Optical flow sensor | SPI |
| NEO-6M GPS | Position, altitude & speed | UART |
| VL53L1X | Time-of-flight distance sensor | I2C |
| ST7735 TFT | 128x160 telemetry display | SPI |

---

## Implemented Features

### 1. Attitude Estimation
- Reads accelerometer and gyroscope data from the BMI160.
- Calculates roll and pitch using a complementary filter.
- Combines gyroscope integration with accelerometer measurements to reduce drift.
- Displays attitude using a graphical artificial horizon.

### 2. Compass & Heading
- Reads three-axis magnetic field measurements from the QMC5883P.
- Applies magnetometer calibration offsets and scaling.
- Calculates heading using calibrated X/Y magnetic field measurements.
- Displays heading through a graphical compass.

### 3. Barometric Altitude
- Reads atmospheric pressure and temperature from the BMP280.
- Estimates relative altitude using an initial pressure reference.
- Displays altitude in meters and pressure in hPa.

### 4. Optical Flow
- Reads X/Y motion counts from the PMW3901.
- Implements noise reduction using:
  - Deadzone filtering
  - Spike rejection
  - Exponential smoothing
- Displays filtered motion measurements.

### 5. GPS Telemetry
- Receives GPS data through UART.
- Parses GPS messages using TinyGPSPlus.
- Displays:
  - Satellite count
  - Latitude and longitude
  - GPS altitude
  - Ground speed
  - GPS fix status

### 6. Distance Measurement
- Interfaces with the VL53L1X time-of-flight sensor.
- Uses long-distance measurement mode.
- Implements median filtering and exponential smoothing.
- Displays filtered distance measurements in centimeters.
- Monitors measurement validity and data freshness.

### 7. TFT Telemetry Dashboard
A seven-page graphical interface with push-button navigation.

| Page | Display |
|------|---------|
| 1 | Drone Attitude |
| 2 | Magnetometer |
| 3 | Barometric Altitude |
| 4 | Optical Flow |
| 5 | System Status |
| 6 | GPS Telemetry |
| 7 | Distance Sensor |

---

## GPIO Configuration

| Component | ESP32-S3 GPIO |
|-----------|--------------|
| Main I2C SDA | 21 |
| Main I2C SCL | 20 |
| VL53L1X SDA | 4 |
| VL53L1X SCL | 5 |
| TFT MOSI | 11 |
| TFT SCK | 12 |
| TFT CS | 8 |
| TFT DC | 9 |
| TFT RST | 10 |
| PMW3901 MOSI | 3 |
| PMW3901 MISO | 13 |
| PMW3901 SCK | 18 |
| PMW3901 CS | 7 |
| PMW3901 RST | 6 |
| GPS RX | 44 |
| GPS TX | 43 |
| Button UP | 1 |
| Button DOWN | 2 |

---

## Software Dependencies

Developed using the Arduino framework for ESP32-S3.

Required libraries:

- Arduino Core for ESP32
- Wire
- SPI
- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library
- Adafruit BMP280 Library
- Adafruit QMC5883P Library
- Bitcraze PMW3901
- TinyGPSPlus
- VL53L1X

The BMI160 is accessed directly through I2C register operations.

---

## Sensor Update Intervals

| Sensor / System | Configured Interval |
|-----------------|---------------------|
| BMI160 | 10 ms |
| QMC5883P | 50 ms |
| BMP280 | 100 ms |
| PMW3901 | 50 ms |
| VL53L1X | 50 ms continuous ranging |
| TFT Display | 50 ms |

These values represent configured scheduling intervals rather than guaranteed execution rates.

---

## Firmware Architecture

The firmware uses a non-blocking, `millis()`-based scheduling approach for periodic sensor updates.

Each sensor has a dedicated update function, allowing measurements to be processed independently.

### Sensor Processing

```text
ESP32-S3
   |
   +-- BMI160 --------> Roll / Pitch Estimation
   |
   +-- QMC5883P ------> Compass Heading
   |
   +-- BMP280 --------> Relative Altitude
   |
   +-- PMW3901 -------> Optical Flow
   |
   +-- NEO-6M GPS ----> Position / Speed
   |
   +-- VL53L1X -------> Distance Measurement
   |
   +-- ST7735 TFT ----> Telemetry Dashboard
```

### Data Processing Methods

- Complementary filtering for attitude estimation
- Magnetometer offset and scale calibration
- Exponential smoothing for optical flow
- Median and exponential filtering for distance measurements
- Sensor initialization and status monitoring

---

## Development Roadmap

### Current Development
- [x] ESP32-S3 sensor interfacing
- [x] BMI160 accelerometer and gyroscope integration
- [x] Roll and pitch estimation
- [x] Magnetometer calibration and heading calculation
- [x] Barometric altitude estimation
- [x] Optical flow signal filtering
- [x] GPS data parsing
- [x] Time-of-flight distance filtering
- [x] TFT telemetry dashboard
- [x] Sensor initialization status monitoring

### Planned Development
- [ ] Motor and ESC control
- [ ] Flight stabilization and PID control
- [ ] Raspberry Pi communication
- [ ] Computer vision integration
- [ ] Object recognition and tracking
- [ ] Autonomous target-following logic
- [ ] Sensor fusion improvements
- [ ] Flight testing and validation

---

## Project Goal

The goal of this project is to develop a custom autonomous UAV that integrates embedded systems, sensor processing, flight control, and computer vision.

The ESP32-S3 will support real-time hardware interfacing and low-level control, while the Raspberry Pi will handle high-level perception and target-following operations.

**Status:** Work in progress.
