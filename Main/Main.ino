/*
 * ENES100 – WATER Mission – Mecanum OTV
 * University of Maryland – Tidal Terps
 *
 * What this does:
 * - Connects to ENES100 Vision System (Wi-Fi module pins set below)
 * - Reads OTV pose (x, y, theta) from overhead system
 * - Demo forward motion + ~90° turns (MS5 basics)
 * - Obstacle sensor calibration + simple avoidance (strafe)
 * - Mecanum holonomic drive (vx, vy, omega) with simple P control to waypoints
 * - WATER mission sensing: depth (mm) + water type (fresh polluted/unpolluted)
 * - Submits mission results with Enes100.mission()
 *
 * Replace STUBS with your actual sensors/driver specifics as you integrate.
 */

#include <Arduino.h>
#include <math.h>
#include "Enes100.h"

// ========= TEAM / VISION CONFIG (EDIT THESE) =========
static const char* TEAM_NAME   = "Tidal Terps"; // must match ML model name if you use ML
static const byte  TEAM_TYPE   = WATER;         // Mission: WATER
static const int   MARKER_ID   = 3;             // <-- your ArUco marker ID
static const int   ROOM_NUMBER = 1120;          // 1120 or 1116

// Wi-Fi module pins (Arduino <-> ESP8266/ESPCAM). Use allowed pins for your board.
static const int WIFI_TX_PIN = 8;  // Arduino -> WiFi RX
static const int WIFI_RX_PIN = 9;  // Arduino <- WiFi TX

// ========= MECHANUM MOTOR PINS (EDIT IF NEEDED) =========
// If your driver needs DIR pins, add them and apply sign before PWM.
const uint8_t FL_PWM = 3;  // Front-left
const uint8_t FR_PWM = 5;  // Front-right
const uint8_t BL_PWM = 6;  // Back-left
const uint8_t BR_PWM = 9;  // Back-right

// ========= OBSTACLE & MISSION SENSORS (EDIT FOR YOUR HARDWARE) =========
const uint8_t OBST_SENSOR_PIN  = A0; // e.g., IR/sonar analog voltage (for demo)
const uint8_t DEPTH_SENSOR_PIN = A1; // e.g., pressure sensor (analog)
const uint8_t TURB_SENSOR_PIN  = A2; // e.g., turbidity (analog)

// Tunables (calibrate on your OTV)
float g_obstBaseline               = 0.0f;
float g_obstAvoidThreshVolts       = 0.25f;     // delta from baseline indicating obstacle
int   g_depthZeroOffsetCounts      = 0;         // ADC offset at 0 mm
float g_depthScale_mm_per_count    = 0.50f;     // mm per ADC count (calibrate!)
int   g_turbidityThresholdCounts   = 500;       // polluted vs not (calibrate!)

// ========= NAV / CONTROL TUNING =========
static const float    kMaxCmd      = 1.0f;    // clamp for vx, vy, omega
static const float    kVTransGain  = 0.7f;    // P gain for translation
static const uint16_t kLoopDtMs    = 20;      // loop period
static const float    kArriveTol   = 0.05f;   // meters

// ========= FIELD TARGETS (SET TO YOUR ARENA) =========
volatile float TARGET_MISSION_X = 3.00f; // m
volatile float TARGET_MISSION_Y = 1.00f; // m
volatile float TARGET_LIMBO_X   = 3.40f; // m
volatile float TARGET_LIMBO_Y   = 0.50f; // m

// ========= STATE MACHINE =========
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

// ========= MATH HELPERS =========
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float hypot2(float dx, float dy) { return sqrtf(dx*dx + dy*dy); }
static inline float normalizeAngle(float a) { while (a < 0) a += 2*PI; while (a >= 2*PI) a -= 2*PI; return a; }

// ========= POSE WRAPPER (FIXED) =========
struct VisionPose {
  float x, y, theta; // meters, meters, radians (-pi..pi from ENES100)
  bool  visible;
};
// cache last good pose to ride out brief occlusions
static VisionPose g_lastGoodPose = {NAN, NAN, 0.0f, false};

static VisionPose readPoseVision() {
  VisionPose p;
  p.x      = Enes100.getX();
  p.y      = Enes100.getY();
  p.theta  = Enes100.getTheta();
  p.visible= Enes100.isVisible();

  if (p.visible && isfinite(p.x) && isfinite(p.y)) {
    g_lastGoodPose = p;
    return p;
  }
  // fallback to last good (may be invalid at boot until first lock)
  return g_lastGoodPose;
}

static bool arrived(float targetX, float targetY, const VisionPose& p) {
  if (!isfinite(p.x) || !isfinite(p.y)) return false; // no valid pose yet
  return hypot2(targetX - p.x, targetY - p.y) <= kArriveTol;
}

// ========= ENES100 LOGGING =========
static void missionPrintln(const String& msg) { Enes100.println(msg); }

// ========= MOTOR I/O =========
static void setMotorPWM(uint8_t pin, int pwm) {
  analogWrite(pin, constrain(pwm, 0, 255));
}
static void stopMotors() {
  // 128 ~ neutral in this simple mapping; change if your ESC/H-bridge differs
  setMotorPWM(FL_PWM, 128);
  setMotorPWM(FR_PWM, 128);
  setMotorPWM(BL_PWM, 128);
  setMotorPWM(BR_PWM, 128);
}

// Mecanum: (vx, vy, omega) in [-1,1] -> four wheel PWMs
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

// ========= OBSTACLE SENSING & CAL =========
static float readObstacleVolts() {
  int raw = analogRead(OBST_SENSOR_PIN);
  return (float)raw * (5.0f / 1023.0f);
}
static bool obstacleDetected() {
  float v = readObstacleVolts();
  return isfinite(g_obstBaseline) && (fabsf(v - g_obstBaseline) > g_obstAvoidThreshVolts);
}
static void obstacleAvoidStep() {
  // Simple evasive: strafe left; tune as needed
  driveMecanum(0.0f, -0.7f, 0.0f);
}

static void calibrateObstacleSensor(uint16_t samples = 120) {
  float acc = 0.0f;
  for (uint16_t i = 0; i < samples; ++i) { acc += readObstacleVolts(); delay(5); }
  g_obstBaseline = acc / samples;
  missionPrintln(String("Obstacle baseline V=") + String(g_obstBaseline, 3));
}

static void calibrateDepthZero(uint16_t samples = 120) {
  long acc = 0;
  for (uint16_t i = 0; i < samples; ++i) { acc += analogRead(DEPTH_SENSOR_PIN); delay(5); }
  g_depthZeroOffsetCounts = acc / samples;
  missionPrintln(String("Depth zero offset=") + g_depthZeroOffsetCounts);
}

static void calibrateTurbidity(uint16_t samples = 120) {
  long acc = 0;
  for (uint16_t i = 0; i < samples; ++i) { acc += analogRead(TURB_SENSOR_PIN); delay(5); }
  g_turbidityThresholdCounts = acc / samples; // simple midpoint; refine with clean/dirty refs
  missionPrintln(String("Turbidity thresh=") + g_turbidityThresholdCounts);
}

// ========= WATER MISSION SENSING =========
static int readDepthMM() {
  int raw = analogRead(DEPTH_SENSOR_PIN) - g_depthZeroOffsetCounts;
  int mm  = (int) lroundf(raw * g_depthScale_mm_per_count);
  return max(0, mm);
}
static bool isPolluted() {
  int raw = analogRead(TURB_SENSOR_PIN);
  // flip the comparison if your sensor polarity is inverted
  return raw >= g_turbidityThresholdCounts;
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

// ========= DEMO STEPS =========
static void demoForwardStep() { driveMecanum(0.6f, 0.0f, 0.0f); } // forward
static void demoTurn90Step()  { driveMecanum(0.0f, 0.0f, 0.6f); } // spin

// ========= NAV HELPERS =========
static void translateToward(float x, float y, const VisionPose& p) {
  if (!isfinite(p.x) || !isfinite(p.y)) { stopMotors(); return; }
  float dx = x - p.x;
  float dy = y - p.y;
  float vx = kVTransGain * dx;
  float vy = kVTransGain * dy;
  float mag = hypot2(vx, vy);
  if (mag > kMaxCmd) { vx *= (kMaxCmd/mag); vy *= (kMaxCmd/mag); }
  driveMecanum(vx, vy, 0.0f);
}

// ========= SETUP / LOOP =========
static unsigned long t0 = 0;
static uint8_t turnCount = 0;

void setup() {
  Serial.begin(9600);

  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
  pinMode(OBST_SENSOR_PIN, INPUT);
  pinMode(DEPTH_SENSOR_PIN, INPUT);
  pinMode(TURB_SENSOR_PIN, INPUT);

  // Blocks until Vision System is connected
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
      calibrateObstacleSensor();
      calibrateDepthZero();
      calibrateTurbidity();
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

      if (obstacleDetected()) { missionPrintln("Obstacle detected – avoidance step"); obstacleAvoidStep(); break; }

      translateToward(TARGET_MISSION_X, TARGET_MISSION_Y, p);
      if (arrived(TARGET_MISSION_X, TARGET_MISSION_Y, p)) {
        stopMotors(); missionPrintln("Arrived at mission site.");
        g_state = RunState::SAMPLE_WATER; t0 = millis();
      }
    } break;

    case RunState::SAMPLE_WATER: {
      if (millis() - t0 < 500) { stopMotors(); break; } // settle
      sendWaterResults();
      missionPrintln("Water results sent.");
      g_state = RunState::NAV_TO_LIMBO;
    } break;

    case RunState::NAV_TO_LIMBO: {
      VisionPose p = readPoseVision();
      if (!isfinite(p.x)) { stopMotors(); break; }

      if (obstacleDetected()) { missionPrintln("Obstacle detected – avoidance step"); obstacleAvoidStep(); break; }

      translateToward(TARGET_LIMBO_X, TARGET_LIMBO_Y, p);
      if (arrived(TARGET_LIMBO_X, TARGET_LIMBO_Y, p)) {
        stopMotors(); missionPrintln("Reached pre-limbo staging point.");
        g_state = RunState::HOLD;
      }
    } break;

    case RunState::HOLD: {
      stopMotors(); // idle
    } break;
  }

  if (!visionOK && g_state >= RunState::NAV_TO_MISSION) {
    stopMotors();
    g_state = RunState::WAIT_VISION;
  }

  delay(kLoopDtMs);
}