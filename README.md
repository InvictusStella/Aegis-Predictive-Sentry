# **Aegis Predictive Sentry System**

> An automated, predictive target-tracking sentry using ESP32 and IoT analytics.

## **1. Project Overview**
The **Aegis Predictive Sentry System** is an automated target tracking project based on the ESP32. It uses three outward-facing PIR sensors to detect broad motion events (Front=0°, Right=90°, Left=270°). When motion is registered, the sentry aims a motor-mounted Time-of-Flight (ToF) distance sensor towards the trigger zone. Once localized, the sentry engages its tracking logic, calculating linear velocity and tracking targets predictively.

Locally, visual feedback is displayed on an onboard radar-like OLED screen. Simultaneously, telemetry and tracking metrics are synchronized to the **Arduino Cloud**, allowing remote monitoring of target velocity, hit ratios, and sweeping angles.

---

## **2. Key Features**
* **360-Degree Motion Detection:** Uses three PIR sensors covering 0°, 90°, and 270° for broad event triggers.
* **Precision Pinpointing:** Utilizes a Time-of-Flight (ToF) sensor to calculate exact distance and localization.
* **Predictive Tracking:** Calculates linear velocity and dynamically tracks moving targets predictively.
* **Real-time Radar Feedback:** Displays localized visual readouts on an onboard OLED screen.
* **Cloud Synchronization:** Deep integration with Arduino IoT Cloud for telemetry, false-positive tracking, and target velocity logging.

---

## **3. Hardware Specifications**
| Component | Model | Task |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32 | Central coordination, IoT cloud sync, and sensor logic. |
| **Motion Sensor** | 3x HC-SR501 PIR | Detects broad motion events across three zones (0°, 90°, 270°). |
| **Distance Sensor** | VL53L0X ToF | Pinpoints precise ranging and distance calculation. |
| **Actuator**| 28BYJ-48 Stepper + ULN2003 | Rotates the ToF sensor toward the triggered zone. |
| **Display** | SSD1306 128x64 OLED | Renders real-time radar-like tracking visuals locally. |

---

## **4. System Architecture**
The system architecture revolves around an event-driven loop on the ESP32. Three PIR sensors monitor distinct zones. When a PIR triggers, the system commands the 28BYJ-48 Stepper Motor to rotate the VL53L0X ToF sensor to the active zone. It then scans for accurate distance and calculates the target's linear velocity in millimeters per second. This data allows the ESP32 to predict target movement and aim accordingly. Local status is shown on the SSD1306 OLED, while metrics are pushed continuously to the Arduino IoT Cloud via Wi-Fi.

---

## **5. Dashboard & Analytics**
This project was built and designed exclusively inside the **Arduino IoT Cloud**. It relies on remote variable synchronization to track telemetry and performance ratios. The cloud variables require matching types in your IoT Dashboard:

* **`pirZoneTriggered`** *(Integer)*: Indicates the angle zone of the triggering PIR sensor.
* **`cloudDistance`** *(Integer)*: Holds the current target distance in millimeters.
* **`cloudVelocity`** *(Float)*: Logs the calculated target velocity in mm/s.
* **`cloudPredictedDist`** *(Integer)*: Returns the anticipated future traversal distance.
* **`cloudAngle`** *(Integer)*: Records the current sweeping angle of the tracker.
* **`actuatorAngle`** *(Integer/Float)*: Used to manage/record the precise orientation of the stepper.
* **`cloudConfirmed`** *(Integer)*: Cumulative count of confirmed target locks.
* **`cloudFalsePos`** *(Integer)*: Tracks the number of unverified alarms.
* **`cloudFPRatio`** *(Float)*: The ratio comparing actual targets vs false positive sweeps.

---

## **6. How to Use**
1. **Cloud Setup:** Create the specified variables in your Arduino IoT Cloud dashboard. The Arduino IoT editor will automatically set up `thingProperties.h` matching these variables.
2. **Flash Firmware:** Copy the code from `PredictiveSentry.ino` into your own Arduino Cloud Web Editor Sketch and upload it to the ESP32.
3. **Power On:** Connect the ESP32 to a power source. It will resolve WiFi and Cloud connections.
4. **Monitor:** Trigger a PIR sensor zone. The stepper will aim the ToF sensor, the OLED will display tracking visuals, and your Cloud Dashboard will process the metrics in real time!
