
# UAV Electrical Hardware

## Overview

This directory contains the electrical hardware design and
documentation for a custom thrust-vectoring tail-sitter VTOL UAV.

The electrical system is designed to integrate an ESP32-S3
microcontroller, Raspberry Pi 5, onboard sensors, motor
controllers, and power distribution circuitry.

All electrical schematics are developed using Autodesk
Fusion 360 Electronics.

## Electrical System Architecture

The UAV uses a 6S LiPo battery as its primary power source.

The electrical architecture includes:

- **6S LiPo Battery:** Main power source
- **ACS758:** Battery current monitoring
- **4-in-1 ESC:** Controls four brushless propulsion motors
- **12V Buck Converter:** Power supply for thrust-vectoring motor drivers
- **5V Buck Converter:** Power supply for onboard computing and peripherals
- **3.3V Buck Converter:** Regulated power for compatible sensors
- **ESP32-S3:** Sensor interfacing and embedded control
- **Raspberry Pi 5:** Computer vision and object tracking
- **DRV8313:** Brushless motor drivers for thrust-vectoring actuators
- **AS5600:** Magnetic position feedback for thrust-vectoring mechanisms

## Electrical Schematics

### Initial Electrical System Design

The schematic documents the preliminary integration of
power distribution, embedded controllers, sensors,
propulsion electronics, and thrust-vectoring actuators.

**Status:** Work in Progress

[View Electrical Schematic (PDF)](schematics/UAV_Electrical_Schematic_v1.pdf)

## Development Progress

- [x] Initial electrical system architecture
- [x] Component selection
- [x] Preliminary power distribution schematic
- [x] Sensor interface planning
- [ ] Propulsion and thrust-vectoring wiring design
- [ ] Electrical schematic review and validation
- [ ] Power distribution testing
- [ ] Motor driver testing
- [ ] Hardware integration
- [ ] Full system testing

## Design Tools

- Autodesk Fusion 360 Electronics

## Future Development

- Validate power distribution under load
- Verify voltage regulation and current consumption
- Test ESC and motor control interfaces
- Integrate ESP32-S3 and Raspberry Pi communication
- Perform electrical reliability testing
