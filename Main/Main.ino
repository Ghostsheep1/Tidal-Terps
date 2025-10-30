/*
 * ENES100 – WATER Mission – Mecanum OTV
 * University of Maryland – Tidal Terps
 *
 * - ENES100 Vision System (Wi-Fi pins set below)
 * - Pose wrapper with last-good caching (VisionPose)
 * - Demo forward + ~90° turns
 * - Obstacle avoidance with HC-SR04 (air ultrasonic)
 * - Simple P-control waypoint nav (mecanum)
 * - WATER mission: depth (mm) + water type reporting (stubbed analogs)
 *
 * TODOs for your bot:
 *  - Set MARKER_ID, ROOM_NUMBER, arena targets, and motor pin mapping
 *  - Calibrate depth/turbidity sensors or replace stubs with your real sensors
 */

#include <Arduino.h>
#include <math.h>
#include "Enes100.h"

// ================= TEAM / VISION CONFIG (EDIT) =================
static const char* TEAM_NAME   = "Tidal Terps";
static const byte  TEAM_TYPE   = WATER;      // Mission type
static const int   MARKER_ID   = 3;          // <-- your ArUco ID
static const int   ROOM_NUMBER = 1120;       // 1120 or 1116

// Use allowed pins for your board. Avoid PWM pins used by motors.
// CHANGED: moved Wi-Fi to 10/11 to avoid conflict with motor PWM on 9.
static const int WIFI_TX_PIN = 10;  // Arduino -> Wi-Fi RX
static const int WIFI_RX_PIN = 11;  // Arduino <- Wi-Fi TX

// ================= MECHANUM MOTOR PINS (EDIT IF NEEDED) =================
// If your driver needs DIR pins, add them and sign the PWM accordingly.
const uint8_t FL_PWM = 3;   // Front-left
const uint8_t FR_PWM = 5;   // Front-right
const uint8_t BL_PWM = 6;   // Back-left
const uint8_t BR_PWM = 9;   // Back-right

// ================= OBSTACLE SENSOR: HC-SR04 (TRIG/ECHO) =================
const uint8_t HC_TRIG_PIN = 7;   // digital output
const uint8_t HC_ECHO_PIN = 4;   // digital input (use interrupt-capable pin if you like)

// Obstacle thresholds (meters) — tune to your course
float g_obstBaselineM         = NAN;  // measured at boot
float g_obstTriggerDeltaM     = 0.15; // trigger if < (baseline - delta)
float g_obstMinTriggerM       = 0.35; // or absolute: anything closer than this is an obstacle
unsigned long g_echoTimeoutUs = 25000UL; // ~4.3 m range timeout

// ================= WATER MISSION SENSORS (STUB ANALOGS) =================
// Replace with your real water sensors (these are not the HC-SR04)
const uint8_t DEPTH_SENSOR_PIN = A1; // pressure sensor analog (example)
const uint8_t TURB_SENSOR_PIN  = A2; // turbidity analog (example)

int   g_depthZeroOffsetCounts   = 0;
float g_depthScale_mm_per_count = 0.50f; // calibrate!
int   g_turbidityThresholdCounts= 500;   // calibrate!

// ================= NAV / CONTROL TUNING =================
static const float    kMaxCmd     = 1.0f;   // limit |vx|,|vy|,|omega|
static const float    kVTransGain = 0.7f;   // P gain
static const uint16_t kLoopDtMs   = 20;     // loop period
static const float    kArriveTol  = 0.05f;  // meters

// ================= ARENA TARGETS (EDIT FOR YOUR FIELD) =================
volatile float TARGET_MISSION_X = 3.00f; // m
volatile float TARGET_MISSION_Y = 1.00f; // m
volatile float TARGET_LIMBO_X   = 3.40f; // m
volatile float TARGET_LIMBO_Y   = 0.50f; // m

// ================= STATE MACHINE =================
enum class RunState : uint8_t {
  WAIT_VISION = 0,
  CALIBRATE_SENSORS,
  FORWARD_DEMO,
  TURNING_DEMO,
  NAV_TO_MISSION,
  SAMPLE_WATER,
  NAV_TO_LIMBO,
  HOLD
};
RunState g_state = RunState::WAIT_VISION;

// ================= MATH HELPERS =================
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float hypot2(float dx, float dy) { return sqrtf(dx*dx + dy*dy); }
static inline float normalizeAngle(float a) { while (a < 0) a += 2*PI; while (a >= 2*PI) a -= 2*PI; return a; }

// ================= POSE WRAPPER (VISION + CACHING) =================
struct VisionPose { float x, y, theta; bool visible; };
static VisionPose g_lastGoodPose = {NAN, NAN, 0.0f, false};

static VisionPose readPoseVision() {
  VisionPose p;
  p.x       = Enes100.getX();
  p.y       = Enes100.getY();
  p.theta   = Enes100.getTheta();
  p.visible = Enes100.isVisible();

  if (p.visible && isfinite(p.x) && isfinite(p.y)) {
    g_lastGoodPose = p;
    return p;
  }
  return g_lastGoodPose; // may be invalid until first lock
}

static bool arrived(float tx, float ty, const VisionPose& p) {
  if (!isfinite(p.x) || !isfinite(p.y)) return false;
  return hypot2(tx - p.x, ty - p.y) <= kArriveTol;
}

// ================= LOGGING HELPERS =================
static void missionPrintln(const String& msg) { Enes100.println(msg); }

// ================= MOTOR I/O + MECANUM =================
static void setMotorPWM(uint8_t pin, int pwm) { analogWrite(pin, constrain(pwm, 0, 255)); }
static void stopMotors() {
  setMotorPWM(FL_PWM, 128);
  setMotorPWM(FR_PWM, 128);
  setMotorPWM(BL_PWM, 128);
  setMotorPWM(BR_PWM, 128);
}

static void driveMecanum(float vx, float vy, float omega) {
  vx = clampf(vx, -kMaxCmd, kMaxCmd);
  vy = clampf(vy, -kMaxCmd, kMaxCmd);
  omega = clampf(omega, -kMaxCmd, kMaxCmd);

  const float fl = vx - vy - omega;
  const float fr = vx + vy + omega;
  const float bl = vx + vy - omega;
  const float br = vx - vy + omega;

  auto toPWM = [](float s)->int { s = clampf(s, -1.0f, 1.0f); return (int)lroundf(s * 127.0f + 128.0f); };
  setMotorPWM(FL_PWM, toPWM(fl));
  setMotorPWM(FR_PWM, toPWM(fr));
  setMotorPWM(BL_PWM, toPWM(bl));
  setMotorPWM(BR_PWM, toPWM(br));
}

// ================== HC-SR04 MEASUREMENT ==================
static float hcsr04_readDistanceM_once() {
  // Trigger a 10us pulse
  digitalWrite(HC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG_PIN, LOW);

  // Measure echo pulse width (microseconds). Timeout limits max distance.
  unsigned long dur = pulseIn(HC_ECHO_PIN, HIGH, g_echoTimeoutUs);
  if (dur == 0UL) return NAN; // timeout / out of range

  // distance (m) = (duration_us * speed_of_sound_m/us) / 2 (round trip)
  // speed_of_sound ~ 0.000343 m/us at ~20°C
  return (dur * 0.000343f) * 0.5f;
}

// Take N readings and return the median for robustness.
static float hcsr04_readDistanceM(uint8_t samples = 3) {
  float vals[5];
  samples = constrain(samples, 1, 5);
  uint8_t n = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    float d = hcsr04_readDistanceM_once();
    if (isfinite(d)) vals[n++] = d;
    delay(10);
  }
  if (n == 0) return NAN;
  // simple insertion sort for small n
  for (uint8_t i = 1; i < n; ++i) {
    float key = vals[i]; int j = i - 1;
    while (j >= 0 && vals[j] > key) { vals[j+1] = vals[j]; j--; }
    vals[j+1] = key;
  }
  return vals[n/2]; // median
}

// ================== OBSTACLE DETECT / CAL ==================
static bool obstacleDetected() {
  float d = hcsr04_readDistanceM();
  if (!isfinite(d)) return false; // no reading -> don't panic
  // absolute trigger OR relative vs baseline
  bool absHit = (d <= g_obstMinTriggerM);
  bool relHit = (isfinite(g_obstBaselineM) && d < (g_obstBaselineM - g_obstTriggerDeltaM));
  return absHit || relHit;
}

static void obstacleAvoidStep() {
  // simple evasive: strafe left; tune duration by staying in this state a tick
  driveMecanum(0.0f, -0.7f, 0.0f);
}

static void calibrateHC_SR04(uint16_t samples = 12) {
  float acc = 0.0f; int got = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    float d = hcsr04_readDistanceM();
    if (isfinite(d)) { acc += d; got++; }
    delay(25);
  }
  if (got > 0) {
    g_obstBaselineM = acc / got;
    missionPrintln(String("HC-SR04 baseline = ") + String(g_obstBaselineM, 3) + " m");
  } else {
    g_obstBaselineM = NAN;
    missionPrintln("HC-SR04 baseline: no reading (check wiring/aim)");
  }
}

// ================== WATER MISSION (STUB ANALOGS) ==================
static void calibrateDepthZero(uint16_t samples = 120) {
  long acc = 0;
  for (uint16_t i = 0; i < samples; ++i) { acc += analogRead(DEPTH_SENSOR_PIN); delay(5); }
  g_depthZeroOffsetCounts = acc / samples;
  missionPrintln(String("Depth zero offset=") + g_depthZeroOffsetCounts);
}

static void calibrateTurbidity(uint16_t samples = 120) {
  long acc = 0;
  for (uint16_t i = 0; i < samples; ++i) { acc += analogRead(TURB_SENSOR_PIN); delay(5); }
  g_turbidityThresholdCounts = acc / samples;
  missionPrintln(String("Turbidity thresh=") + g_turbidityThresholdCounts);
}

static int readDepthMM() {
  int raw = analogRead(DEPTH_SENSOR_PIN) - g_depthZeroOffsetCounts;
  int mm  = (int) lroundf(raw * g_depthScale_mm_per_count);
  return max(0, mm);
}

static bool isPolluted() {
  int raw = analogRead(TURB_SENSOR_PIN);
  return raw >= g_turbidityThresholdCounts; // flip if your sensor polarity is reversed
}

static void sendWaterResults() {
  int depth_mm = readDepthMM();
  Enes100.mission(DEPTH, depth_mm);
  if (isPolluted()) {
    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
    missionPrintln("Mission: WATER_TYPE=FRESH_POLLUTED");
  } else {
    Enes100.mission(WATER_TYPE, FRESH_UNPOLLUTED);
    missionPrintln("Mission: WATER_TYPE=FRESH_UNPOLLUTED");
  }
  missionPrintln(String("Mission: DEPTH=") + depth_mm + " mm");
}

// ================== DEMOS + NAV ==================
static void demoForwardStep() { driveMecanum(0.6f, 0.0f, 0.0f); }
static void demoTurn90Step()  { driveMecanum(0.0f, 0.0f, 0.6f); }

static void translateToward(float x, float y, const VisionPose& p) {
  if (!isfinite(p.x) || !isfinite(p.y)) { stopMotors(); return; }
  float dx = x - p.x, dy = y - p.y;
  float vx = kVTransGain * dx, vy = kVTransGain * dy;
  float mag = hypot2(vx, vy);
  if (mag > kMaxCmd) { vx *= (kMaxCmd/mag); vy *= (kMaxCmd/mag); }
  driveMecanum(vx, vy, 0.0f);
}

// ================== SETUP / LOOP ==================
static unsigned long t0 = 0;
static uint8_t turnCount = 0;

void setup() {
  Serial.begin(9600);

  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);

  pinMode(HC_TRIG_PIN, OUTPUT);
  pinMode(HC_ECHO_PIN, INPUT);

  pinMode(DEPTH_SENSOR_PIN, INPUT);
  pinMode(TURB_SENSOR_PIN, INPUT);

  // ENES100 connect (blocks until connected)
  Enes100.begin(TEAM_NAME, TEAM_TYPE, MARKER_ID, ROOM_NUMBER, WIFI_TX_PIN, WIFI_RX_PIN);
  missionPrintln("ENES100 connected. WATER mission start.");

  stopMotors();
  g_state = RunState::CALIBRATE_SENSORS;
  g_lastGoodPose = {NAN, NAN, 0.0f, false};
}

void loop() {
  bool visionOK = Enes100.isConnected();

  switch (g_state) {
    case RunState::WAIT_VISION: {
      if (visionOK) { missionPrintln("Vision reconnected."); g_state = RunState::CALIBRATE_SENSORS; }
    } break;

    case RunState::CALIBRATE_SENSORS: {
      calibrateHC_SR04();         // baseline distance in current environment
      calibrateDepthZero();       // water depth zero
      calibrateTurbidity();       // water turbidity threshold
      t0 = millis();
      g_state = RunState::FORWARD_DEMO;
    } break;

    case RunState::FORWARD_DEMO: {
      demoForwardStep();
      if (millis() - t0 > 2000) {
        stopMotors(); delay(300);
        t0 = millis(); turnCount = 0;
        g_state = RunState::TURNING_DEMO;
      }
    } break;

    case RunState::TURNING_DEMO: {
      demoTurn90Step();
      if (millis() - t0 > 700) { // tune for ~90°
        stopMotors(); delay(300);
        t0 = millis(); turnCount++;
        if (turnCount >= 3) g_state = RunState::NAV_TO_MISSION;
      }
    } break;

    case RunState::NAV_TO_MISSION: {
      VisionPose p = readPoseVision();
      if (!isfinite(p.x)) { missionPrintln("Waiting for first valid pose..."); stopMotors(); break; }

      if (obstacleDetected()) { missionPrintln("Obstacle detected – avoidance"); obstacleAvoidStep(); break; }

      translateToward(TARGET_MISSION_X, TARGET_MISSION_Y, p);
      if (arrived(TARGET_MISSION_X, TARGET_MISSION_Y, p)) {
        stopMotors(); missionPrintln("Arrived at mission site.");
        g_state = RunState::SAMPLE_WATER; t0 = millis();
      }
    } break;

    case RunState::SAMPLE_WATER: {
      if (millis() - t0 < 500) { stopMotors(); break; } // settle sensors
      sendWaterResults();
      missionPrintln("Water results sent.");
      g_state = RunState::NAV_TO_LIMBO;
    } break;

    case RunState::NAV_TO_LIMBO: {
      VisionPose p = readPoseVision();
      if (!isfinite(p.x)) { stopMotors(); break; }

      if (obstacleDetected()) { missionPrintln("Obstacle detected – avoidance"); obstacleAvoidStep(); break; }

      translateToward(TARGET_LIMBO_X, TARGET_LIMBO_Y, p);
      if (arrived(TARGET_LIMBO_X, TARGET_LIMBO_Y, p)) {
        stopMotors(); missionPrintln("Reached pre-limbo staging point.");
        g_state = RunState::HOLD;
      }
    } break;

    case RunState::HOLD: {
      stopMotors();
    } break;
  }

  if (!visionOK && g_state >= RunState::NAV_TO_MISSION) {
    stopMotors();
    g_state = RunState::WAIT_VISION;
  }

  delay(kLoopDtMs);
}
