# Aegis Predictive Sentry System

## Overview
The **Aegis Predictive Sentry System** is an automated target tracking project based on the ESP32. It uses three outward-facing PIR sensors to detect broad motion events (Front=0°, Right=90°, Left=270°). When motion is registered, the sentry aims a motor-mounted Time-of-Flight (ToF) distance sensor towards the trigger zone. Once localized, the sentry engages its tracking logic, calculating linear velocity and tracking targets predictively. Visual feedback is displayed on an onboard radar-like OLED screen. **Do consider** that this is a basic project aimed at learning to work with these components and technologies rather than a full-blown technological breakthrough, if you think it would be fun or useful for your coursework please be my guest at taking the code and reusing as you see fit. Do be warned that the code was written with the use of AI agents, i did not write this code from stracth myself. 

### Hardware Components
- **ESP32** Microcontroller
- **3x HC-SR501 PIR Sensors** for broad motion detection
- **VL53L0X ToF (Time-of-Flight) Sensor** for pinpoint ranging and distance calculation
- **28BYJ-48 Stepper Motor + ULN2003 Driver** to rotate the ToF sensor
- **SSD1306 128x64 OLED Display** for localized visual readout

## ☁️ Arduino IoT Cloud Setup Instruction

This project was built and designed exclusively inside the **Arduino IoT Cloud**. It takes advantage of its remote variable synchronization. If you intend to use this code, **you must copy the code from `PredictiveSentry.ino` into your own Arduino Cloud Web Editor Sketch**. 

### Cloud Variables Required
Because the logic actively queries and updates your cloud environment, it requires a "Thing" equipped with a specific set of variables. You MUST create the following variables in your Arduino Cloud Dashboard before uploading your sketch to prevent compilation errors involving missing declarations:

- `pirZoneTriggered` *(Integer)*: Indicates the angle zone of the triggering PIR sensor.
- `cloudDistance` *(Integer)*: Holds the current target distance in millimeters.
- `cloudVelocity` *(Float/Floating Point)*: Logs the calculated target velocity in mm/s.
- `cloudPredictedDist` *(Integer)*: Returns the anticipated future traversal distance.
- `cloudAngle` *(Integer)*: Records the current sweeping angle of the tracker.
- `actuatorAngle` *(Integer / Float)*: Used to manage or record the precise geometric orientation of the tracking stepper.
- `cloudConfirmed` *(Integer)*: Cumulative count of confirmed target locks.
- `cloudFalsePos` *(Integer)*: Tracks the number of unverified alarms.
- `cloudFPRatio` *(Float/Floating Point)*: The mathematical ratio comparing actual targets vs false positive sweeps.

The Arduino IoT editor will automatically handle setting up `thingProperties.h` matching these variables, ensuring the main code functions flawlessly.
