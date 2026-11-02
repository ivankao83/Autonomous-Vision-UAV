
# Autonomous Thrust-Vectoring Tail-Sitter UAV

## Project Overview

This project focuses on the design and development of a
custom thrust-vectoring tail-sitter VTOL unmanned aerial vehicle.

The aircraft is designed to take off and hover vertically,
transition into horizontal cruise flight, and utilize
thrust vectoring for enhanced maneuverability and
attitude control.

The UAV integrates an ESP32-S3 microcontroller for
embedded hardware control and a Raspberry Pi 5 for
computer vision, object recognition, and autonomous
target-following capabilities.

The project involves electrical system design,
embedded programming, mechanical design, and
autonomous flight system development.

**Project Status:** In Progress

---

## Project Objectives

- Design and manufacture a custom tail-sitter UAV.
- Develop a thrust-vectoring propulsion system.
- Implement vertical hover and horizontal cruise modes.
- Develop flight stabilization and transition control.
- Integrate computer vision for object detection and tracking.
- Enable autonomous target-following capabilities.
- Design and validate an onboard electrical power system.

---

## System Architecture

The UAV uses two main processing platforms:

### ESP32-S3 — Embedded Control

Responsible for:

- Onboard sensor interfacing
- IMU-based attitude estimation
- GPS and optical flow processing
- Distance measurement
- Real-time telemetry visualization
- Planned flight stabilization and motor control

### Raspberry Pi 5 — High-Level Processing

Planned responsibilities:

- Camera image processing
- Object recognition and detection
- Visual target tracking
- Autonomous target-following command generation
- Communication with the ESP32-S3

---

## Hardware Components

| Component | Purpose |
|-----------|---------|
| ESP32-S3 | Embedded control and sensor processing |
| Raspberry Pi 5 | Computer vision and object tracking |
| 6S LiPo Battery | Primary power source |
| 4-in-1 ESC | Propulsion motor control |
| DRV8313 | Thrust-vectoring motor driver |
| AS5600 | Magnetic angle sensing |
| BMI160 | Accelerometer and gyroscope |
| QMC5883P | Magnetometer |
| BMP280 | Barometric altitude estimation |
| PMW3901 | Optical flow |
| NEO-6M GPS | Position and speed |
| VL53L1X | Distance measurement |

---

## Mechanical Design

The UAV features a custom 3D-printed fuselage designed
using Autodesk Fusion 360.

The mechanical design includes:

- Custom fuselage geometry
- Internal structural reinforcement
- Electronics mounting provisions
- Integrated ventilation and cooling
- Custom propulsion arm assemblies
- Thrust-vectoring mechanism development

### Mechanical Prototype

<img src="mechanical/images/drone-assembly.jpg" width="450" alt="UAV Mechanical Prototype">

*Initial mechanical prototype showing the custom
3D-printed fuselage and propulsion arm assembly.*

[View Mechanical Design & Progress](mechanical/README.md)

---

## Electrical Hardware

The UAV electrical architecture is designed around
a 6S LiPo battery supplying multiple regulated
voltage rails.

The electrical system includes:

- Battery current monitoring
- 12V, 5V, and 3.3V voltage regulation
- ESP32-S3 and Raspberry Pi power distribution
- Propulsion ESC connections
- Thrust-vectoring actuator drivers
- Onboard sensor interfaces

### Electrical Schematic

[View Electrical System Schematic (PDF)](hardware/schematics/UAV_Electrical_Schematic_v1.pdf)

[View Hardware Documentation](hardware/README.md)

---

## Firmware Development

The ESP32-S3 firmware currently implements:

- BMI160 accelerometer and gyroscope interfacing
- Roll and pitch estimation using a complementary filter
- Magnetometer calibration and heading calculations
- Barometric altitude estimation
- Optical flow signal filtering
- GPS data acquisition and parsing
- Time-of-flight distance measurement and filtering
- Seven-page TFT telemetry dashboard
- Sensor initialization and status monitoring

[View Firmware Documentation](firmware/README.md)

---

## Development Progress

### Mechanical
- [x] Initial CAD design
- [x] Fuselage prototype fabrication
- [x] Internal mounting structures
- [x] Cooling system installation
- [ ] Complete propulsion assembly
- [ ] Thrust-vectoring mechanism validation

### Electrical
- [x] Component selection
- [x] Initial electrical architecture
- [x] Preliminary schematic design
- [ ] Power distribution testing
- [ ] Full electrical integration

### Embedded Systems
- [x] Sensor interface development
- [x] Basic attitude estimation
- [x] Optical flow processing
- [x] GPS telemetry
- [x] TFT telemetry dashboard
- [ ] Flight stabilization controller
- [ ] ESC and actuator control integration

### Autonomous Systems
- [ ] Raspberry Pi computer vision integration
- [ ] Object recognition and tracking
- [ ] ESP32-S3 / Raspberry Pi communication
- [ ] Autonomous target-following logic
- [ ] Hover-to-cruise transition testing
- [ ] Autonomous flight testing

---

## Development Tools

**Mechanical Design**
- Autodesk Fusion 360
- FDM 3D Printing

**Electrical Design**
- Autodesk Fusion 360 Electronics
- Bench Power Supply
- Digital Multimeter

**Embedded Development**
- Arduino Framework
- C/C++
- ESP32-S3

**Computer Vision (Planned)**
- Raspberry Pi 5
- Python
- Camera-based Object Detection

---

## Repository Structure

```text
.
├── README.md
├── firmware/
│   ├── README.md
│   └── drone_telemetry.ino
├── hardware/
│   ├── README.md
│   └── schematics/
│       └── UAV_Electrical_Schematic_v1.pdf
├── mechanical/
│   ├── README.md
│   └── images/
└── computer-vision/
    └── README.md
```

---

## Future Development

The next stages of development focus on completing
mechanical integration, validating the electrical system,
and implementing closed-loop flight control.

Future work will include thrust-vectoring control,
hover-to-cruise transitions, and Raspberry Pi-based
autonomous object tracking.
