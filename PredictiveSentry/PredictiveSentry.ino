/*
 * ============================================================
 *  AEGIS PREDICTIVE SENTRY SYSTEM
 *  Muhammed Kaan Uman
 * ============================================================
 *  HARDWARE:
 *    - ESP32
 *    - 3x HC-SR501 PIR Sensors (Fixed: Front=0°, Right=90°, Left=270°)
 *    - VL53L0X ToF Sensor (on stepper, rotates for tracking)
 *    - 28BYJ-48 Stepper Motor + ULN2003 Driver
 *    - SSD1306 128x64 OLED Display
 *    - Arduino IoT Cloud
 *
 *  LOGIC FLOW:
 *    STANDBY → PIR detects motion → identify zone →
 *    rotate ToF to zone → confirm & lock →
 *    calculate velocity vector → predictive track → cloud telemetry
 * ============================================================
 */

// ── Arduino IoT Cloud ──────────────────────
#include "thingProperties.h"

// ── Libraries ──────────────────────────────────────────────
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Stepper.h>

// ══════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ══════════════════════════════════════════════════════════

// PIR Sensors (digital input)
#define PIR_FRONT   34   // 0°   zone
#define PIR_RIGHT   35   // 90°  zone
#define PIR_LEFT    32   // 270° zone

// 28BYJ-48 Stepper via ULN2003 (IN1–IN4)
#define STEPPER_IN1 25
#define STEPPER_IN2 26
#define STEPPER_IN3 27
#define STEPPER_IN4 14

// OLED – I2C shared with ToF
// SDA = GPIO 21, SCL = GPIO 22 (ESP32 default)

// ══════════════════════════════════════════════════════════
//  CONSTANTS
// ══════════════════════════════════════════════════════════

#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDR       0x3C

#define TOF_ADDR        0x29

// 28BYJ-48: 2048 steps = 360°  (half-step mode with Stepper lib uses 4096/rev sometimes)
// Using the standard Arduino Stepper library at full-step: 2048 steps/rev
#define STEPS_PER_REV   2048
#define STEPPER_RPM     10       // slow for accuracy

// Zone angles (degrees)
#define ANGLE_FRONT     0
#define ANGLE_RIGHT     90
#define ANGLE_LEFT      270

// Tracking parameters
#define TRACK_SAMPLES       5    // samples to average for velocity
#define SAMPLE_INTERVAL_MS  150  // ms between ToF readings
#define PREDICT_HORIZON_MS  300  // how far ahead to predict position
#define LOCK_DISTANCE_MM    3000 // ignore readings beyond 3 m
#define STANDBY_TIMEOUT_MS  5000 // return to standby if no motion

// ══════════════════════════════════════════════════════════
//  SYSTEM STATE MACHINE
// ══════════════════════════════════════════════════════════

enum SentryState {
  STATE_STANDBY,      // Low-power wait for PIR
  STATE_ORIENT,       // Rotate ToF to triggered PIR zone
  STATE_LOCK,         // ToF confirms target distance
  STATE_TRACK,        // Kinematic predictive tracking
  STATE_RETURN        // Return to standby angle
};

SentryState sysState = STATE_STANDBY;

// ══════════════════════════════════════════════════════════
//  OBJECTS
// ══════════════════════════════════════════════════════════

Adafruit_VL53L0X tof;
Adafruit_SSD1306  oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Stepper           stepper(STEPS_PER_REV, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

// ══════════════════════════════════════════════════════════
//  RUNTIME VARIABLES
// ══════════════════════════════════════════════════════════

// Stepper position tracking
int  currentAngle    = 0;   // degrees, 0–359
int  targetAngle     = 0;

// Kinematics ring buffer
struct Sample {
  float    distMM;
  uint32_t timeMs;
};
Sample   samples[TRACK_SAMPLES];
uint8_t  sampleIdx      = 0;
bool     bufferFull     = false;

// Derived kinematics
float    currentDistMM  = 0;
float    velocityMMs    = 0;   // mm/s (+ = moving away, - = approaching)
float    predictedDistMM= 0;

// False-positive tracking
uint32_t falsePositives = 0;
uint32_t confirmedHits  = 0;

// Timing
uint32_t lastSampleTime = 0;
uint32_t lastMotionTime = 0;

// Which PIR zone triggered
int      activeZoneAngle = ANGLE_FRONT;

void setup() {
  Serial.begin(115200);
  Serial.println(F("[ AEGIS SENTRY ] Booting..."));

  // ── GPIO ───────────────────────────────────────────────
  pinMode(PIR_FRONT, INPUT);
  pinMode(PIR_RIGHT, INPUT);
  pinMode(PIR_LEFT,  INPUT);

  // ── Stepper ────────────────────────────────────────────
  stepper.setSpeed(STEPPER_RPM);

  // ── I2C ────────────────────────────────────────────────
  Wire.begin();

  // ── OLED ───────────────────────────────────────────────
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[ERR] OLED init failed"));
    while (true);
  }
  oled.clearDisplay();
  drawBootScreen();

  // ── ToF ────────────────────────────────────────────────
  if (!tof.begin()) {
    Serial.println(F("[ERR] VL53L0X init failed"));
    oledError("TOF FAIL");
    while (true);
  }
  tof.startRangingContinuous(SAMPLE_INTERVAL_MS);
  Serial.println(F("[ AEGIS SENTRY ] Ready."));

  // ── Arduino IoT Cloud ──────────────────────────────────
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  delay(2000);
  drawStandbyScreen();
}

void loop() {
  ArduinoCloud.update();   // keep cloud in sync

  switch (sysState) {

    // ── STANDBY ──────────────────────────────────────────
    case STATE_STANDBY:
      pollPIR();
      break;

    // ── ORIENT ───────────────────────────────────────────
    case STATE_ORIENT:
      rotateTo(activeZoneAngle);
      sysState = STATE_LOCK;
      break;

    // ── LOCK ─────────────────────────────────────────────
    case STATE_LOCK: {
      VL53L0X_RangingMeasurementData_t meas;
      tof.rangingTest(&meas, false);
      if (meas.RangeStatus != 4 && meas.RangeMilliMeter < LOCK_DISTANCE_MM) {
        currentDistMM = meas.RangeMilliMeter;
        confirmedHits++;
        resetKinematics();
        lastMotionTime = millis();
        Serial.printf("[LOCK] Target @ %.0f mm  Zone: %d°\n",
                      currentDistMM, activeZoneAngle);
        sysState = STATE_TRACK;
      } else {
        falsePositives++;
        Serial.println(F("[LOCK] No target confirmed – false positive"));
        updateCloudFP();
        sysState = STATE_RETURN;
      }
      break;
    }

    // ── TRACK ─────────────────────────────────────────────
    case STATE_TRACK:
      runTracker();
      // If no valid reading for STANDBY_TIMEOUT_MS → give up
      if (millis() - lastMotionTime > STANDBY_TIMEOUT_MS) {
        Serial.println(F("[TRACK] Target lost – returning to standby"));
        sysState = STATE_RETURN;
      }
      break;

    // ── RETURN ────────────────────────────────────────────
    case STATE_RETURN:
      rotateTo(ANGLE_FRONT);
      drawStandbyScreen();
      sysState = STATE_STANDBY;
      break;
  }
}

// ══════════════════════════════════════════════════════════
//  PIR POLLING
// ══════════════════════════════════════════════════════════

void pollPIR() {
  bool front = digitalRead(PIR_FRONT);
  bool right = digitalRead(PIR_RIGHT);
  bool left  = digitalRead(PIR_LEFT);

  if (!front && !right && !left) {
    drawStandbyScreen();
    return;
  }

  // Priority: front > right > left (closest to sentry centre first)
  if (front) {
    activeZoneAngle = ANGLE_FRONT;
    Serial.println(F("[PIR] FRONT triggered"));
  } else if (right) {
    activeZoneAngle = ANGLE_RIGHT;
    Serial.println(F("[PIR] RIGHT triggered"));
  } else {
    activeZoneAngle = ANGLE_LEFT;
    Serial.println(F("[PIR] LEFT triggered"));
  }

  // Update cloud variable (defined in thingProperties.h)
  pirZoneTriggered = activeZoneAngle;

  drawOrientScreen(activeZoneAngle);
  sysState = STATE_ORIENT;
}

// ══════════════════════════════════════════════════════════
//  KINEMATIC TRACKER
// ══════════════════════════════════════════════════════════

void runTracker() {
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = millis();

  VL53L0X_RangingMeasurementData_t meas;
  tof.rangingTest(&meas, false);

  if (meas.RangeStatus == 4 || meas.RangeMilliMeter >= LOCK_DISTANCE_MM) {
    // Target momentarily out of range – don't update
    return;
  }

  lastMotionTime = millis();
  currentDistMM  = meas.RangeMilliMeter;

  // Store sample
  samples[sampleIdx] = { (float)currentDistMM, millis() };
  sampleIdx = (sampleIdx + 1) % TRACK_SAMPLES;
  if (sampleIdx == 0) bufferFull = true;

  // Calculate velocity via linear regression over ring buffer
  calcVelocity();

  // Predict future position
  predictedDistMM = currentDistMM + velocityMMs * (PREDICT_HORIZON_MS / 1000.0f);
  predictedDistMM = constrain(predictedDistMM, 0, LOCK_DISTANCE_MM);

  // Steer stepper toward predicted Cartesian X (simplified 1-D tracking)
  // The ToF gives radial distance; velocity sign drives angle compensation
  int angleOffset = calcPredictiveAngleOffset();
  int newAngle    = activeZoneAngle + angleOffset;
  newAngle        = ((newAngle % 360) + 360) % 360;
  rotateTo(newAngle);
  activeZoneAngle = newAngle;  // keep anchor updated

  // Update displays
  updateOLED();
  updateCloud();

  Serial.printf("[TRACK] Dist=%.0fmm  V=%.1fmm/s  PredDist=%.0fmm  Angle=%d°\n",
                currentDistMM, velocityMMs, predictedDistMM, currentAngle);
}

// ── Linear regression velocity over sample buffer ──────────
void calcVelocity() {
  int count = bufferFull ? TRACK_SAMPLES : sampleIdx;
  if (count < 2) { velocityMMs = 0; return; }

  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  uint32_t t0 = samples[0].timeMs;

  for (int i = 0; i < count; i++) {
    float t = (samples[i].timeMs - t0) / 1000.0f;  // seconds
    float d = samples[i].distMM;
    sumX  += t;
    sumY  += d;
    sumXY += t * d;
    sumX2 += t * t;
  }
  float denom = count * sumX2 - sumX * sumX;
  if (fabs(denom) < 1e-6f) { velocityMMs = 0; return; }

  velocityMMs = (count * sumXY - sumX * sumY) / denom;  // mm/s
}

// ── Map velocity to a small angular prediction offset ──────
// Simple heuristic: fast lateral movement → rotate ±15°
int calcPredictiveAngleOffset() {
  // We only have 1-D distance; use velocity magnitude as proxy
  // Positive velocity = moving away, slightly sweep outward
  // Negative velocity = approaching, hold centre
  if (fabs(velocityMMs) < 50) return 0;   // too slow to correct
  int sign = (velocityMMs > 0) ? 1 : -1;
  // Scale: 50 mm/s → ±5°,  200 mm/s → ±15° (capped)
  int offset = (int)(fabs(velocityMMs) / 200.0f * 15.0f);
  offset = constrain(offset, 0, 15);
  return sign * offset;
}

// ══════════════════════════════════════════════════════════
//  STEPPER CONTROL
// ══════════════════════════════════════════════════════════

// Rotate to absolute angle (shortest path)
void rotateTo(int deg) {
  deg = ((deg % 360) + 360) % 360;
  int delta = deg - currentAngle;

  // Shortest path
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;

  int steps = (int)((float)delta / 360.0f * STEPS_PER_REV);
  if (steps != 0) {
    stepper.step(steps);
    currentAngle = deg;
  }
  stepperPowerOff();   // prevent heat buildup when idle
}

// De-energise stepper coils when not moving
void stepperPowerOff() {
  digitalWrite(STEPPER_IN1, LOW);
  digitalWrite(STEPPER_IN2, LOW);
  digitalWrite(STEPPER_IN3, LOW);
  digitalWrite(STEPPER_IN4, LOW);
}

// ══════════════════════════════════════════════════════════
//  OLED DISPLAY
// ══════════════════════════════════════════════════════════

void drawBootScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(20, 10); oled.print(F("AEGIS SENTRY v1.0"));
  oled.setCursor(28, 28); oled.print(F("Akdeniz Univ."));
  oled.setCursor(16, 44); oled.print(F("Initialising HW..."));
  oled.display();
}

void drawStandbyScreen() {
  oled.clearDisplay();
  // Status bar
  oled.setTextSize(1);
  oled.setCursor(0, 0);   oled.print(F("SYS:STANDBY"));
  oled.setCursor(80, 0);  oled.print(F("PIR:WATCH"));
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // Sentry icon (crosshair at centre)
  drawCrosshair(64, 36);

  // Footer
  oled.drawLine(0, 54, 127, 54, SSD1306_WHITE);
  oled.setCursor(0, 56);
  oled.printf("FP:%lu  HIT:%lu", falsePositives, confirmedHits);
  oled.display();
}

void drawOrientScreen(int angle) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print(F("SYS:ORIENT"));
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);
  oled.setCursor(10, 22); oled.print(F("PIR TRIGGERED"));
  oled.setTextSize(2);
  oled.setCursor(30, 38);
  if (angle == ANGLE_FRONT)      oled.print(F("FRONT"));
  else if (angle == ANGLE_RIGHT) oled.print(F("RIGHT"));
  else                           oled.print(F("LEFT"));
  oled.display();
}

void updateOLED() {
  oled.clearDisplay();

  // ── Status bar (top 10 px) ────────────────────────────
  oled.setTextSize(1);
  oled.setCursor(0, 0);   oled.print(F("SYS:TRK_VCTR"));
  oled.setCursor(84, 0);  oled.print(F("WiFi:OK"));
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // ── Dynamic zone (10–54 px) ────────────────────────────
  // Draw room outline (dotted rectangle approximation)
  for (int x = 10; x < 118; x += 4) {
    oled.drawPixel(x,    12, SSD1306_WHITE);
    oled.drawPixel(x,    52, SSD1306_WHITE);
  }
  for (int y = 12; y < 52; y += 4) {
    oled.drawPixel(10,  y, SSD1306_WHITE);
    oled.drawPixel(117, y, SSD1306_WHITE);
  }

  // Origin crosshair
  drawCrosshair(64, 32);

  // Map distance to pixel radius (max 3000mm → 18 px radius)
  float scale = 18.0f / LOCK_DISTANCE_MM;

  // Target ping in polar→Cartesian
  float rad = (float)currentAngle * PI / 180.0f;
  int tx = 64 + (int)(currentDistMM * scale * sin(rad));
  int ty = 32 - (int)(currentDistMM * scale * cos(rad));
  tx = constrain(tx, 12, 116);
  ty = constrain(ty, 14, 50);
  // Draw target circle
  oled.drawCircle(tx, ty, 3, SSD1306_WHITE);

  // Kinematic vector arrow from target
  float prad = (float)currentAngle * PI / 180.0f;
  int px = tx + (int)(velocityMMs * 0.02f * sin(prad));
  int py = ty - (int)(velocityMMs * 0.02f * cos(prad));
  px = constrain(px, 12, 116);
  py = constrain(py, 14, 50);
  oled.drawLine(tx, ty, px, py, SSD1306_WHITE);
  // Arrowhead
  oled.drawPixel(px, py, SSD1306_WHITE);
  oled.drawPixel(px-1, py-1, SSD1306_WHITE);
  oled.drawPixel(px+1, py-1, SSD1306_WHITE);

  // ── Footer (55–63 px) ──────────────────────────────────
  oled.drawLine(0, 54, 127, 54, SSD1306_WHITE);
  oled.setCursor(0, 56);
  oled.printf("TGT:%.0fmm V:%.0fmm/s", currentDistMM, velocityMMs);

  oled.display();
}

void drawCrosshair(int cx, int cy) {
  oled.drawLine(cx-5, cy, cx+5, cy, SSD1306_WHITE);
  oled.drawLine(cx, cy-5, cx, cy+5, SSD1306_WHITE);
  oled.drawCircle(cx, cy, 3, SSD1306_WHITE);
}

void oledError(const char* msg) {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(10, 20);
  oled.print(F("ERROR:"));
  oled.setCursor(10, 40);
  oled.print(msg);
  oled.display();
}

// ══════════════════════════════════════════════════════════
//  CLOUD TELEMETRY
// ══════════════════════════════════════════════════════════

void updateCloud() {
  // Variables declared in thingProperties.h
  cloudDistance     = (int)currentDistMM;
  cloudVelocity     = velocityMMs;
  cloudPredictedDist= (int)predictedDistMM;
  cloudAngle        = currentAngle;
  cloudConfirmed    = confirmedHits;
  cloudFalsePos     = falsePositives;

  float total = falsePositives + confirmedHits;
  cloudFPRatio = (total > 0) ? (falsePositives / total * 100.0f) : 0.0f;
}

void updateCloudFP() {
  cloudFalsePos = falsePositives;
  float total = falsePositives + confirmedHits;
  cloudFPRatio = (total > 0) ? (falsePositives / total * 100.0f) : 0.0f;
}

// ══════════════════════════════════════════════════════════
//  HELPERS
// ══════════════════════════════════════════════════════════

void resetKinematics() {
  sampleIdx   = 0;
  bufferFull  = false;
  velocityMMs = 0;
  for (int i = 0; i < TRACK_SAMPLES; i++) {
    samples[i] = { 0, 0 };
  }
}

void onActuatorAngleChange() {
  // 'actuatorAngle' is a cloud int variable (set from dashboard)
  Serial.printf("[CLOUD] Remote angle command: %d°\n", actuatorAngle);
  rotateTo(actuatorAngle);
}
