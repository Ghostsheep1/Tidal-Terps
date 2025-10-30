/*
 * ENES100 – WATER Mission – Mecanum OTV
 * University of Maryland
 *
 * Covers MS5 sub-tasks: forward motion, turning, wireless RX/TX with Vision System,
 * navigation (no obstacles + avoidance), obstacle sensor calibration, mission sensing
 * (depth + water type), and mission actuation (reporting via Enes100.mission()).
 *
 * Replace STUBS with your real sensors/motor drivers as you integrate.
 * Tested compile target: Arduino Leonardo/Uno-style (adjust pins for Mega/Romeo).
 */

#include <Arduino.h>
#include <math.h>
#include "Enes100.h"

// ========== TEAM / VISION CONFIG ==========
static const char* TEAM_NAME   = "Tidal Terps";   // MUST MATCH your ML model name if you use ML
static const byte  TEAM_TYPE   = WATER;           // Mission type: WATER
static const int   MARKER_ID   = 3;               // <-- your ArUco ID
static const int   ROOM_NUMBER = 1120;            // 1120 or 1116

// WiFi module UART pins (connect to ESP8266/ESPCAM TX/RX)
// Use allowed pins for your board (example uses Leonardo/Uno-safe pins 8/9 from docs)
static const int WIFI_TX_PIN = 8;  // Arduino -> WiFi RX
static const int WIFI_RX_PIN = 9;  // Arduino <- WiFi TX

// ========== MECHANUM MOTOR PINS (PWM) ==========
const uint8_t FL_PWM = 3;  // Front-left
const uint8_t FR_PWM = 5;  // Front-right
const uint8_t BL_PWM = 6;  // Back-left
const uint8_t BR_PWM = 9;  // Back-right
// If you use H-bridges with direction pins, add DIR pins & sign the PWMs accordingly.

// ========== OBSTACLE / MISSION SENSORS ==========
const uint8_t OBST_SENSOR_PIN   = A0; // e.g., analog IR/turbidity/ultrasonic voltage
const uint8_t DEPTH_SENSOR_PIN  = A1; // e.g., pressure sensor analog
const uint8_t TURB_SENSOR_PIN   = A2; // e.g., turbidity analog

// Tunables / thresholds (calibrate!)
float g_obstBaseline      = 0.0f;
float g_obstAvoidThresh   = 0.25f; // volts above/below baseline -> obstacle
int   g_depthZeroOffset   = 0;     // raw ADC offset
float g_depthScale_mm_per_count = 0.50f; // convert raw ADC to mm (calibrate!)
int   g_turbidityThresh   = 500;   // ADC threshold for polluted vs unpolluted (calibrate!)

// ========== NAV / CONTROL TUNING ==========
static const float kMaxCmd       = 1.0f;
static const float kVTransGain   = 0.7f;
static const uint16_t kLoopDtMs  = 20;
static const float kArriveTol    = 0.05f; // meters

// ========== FIELD TARGETS (SET THESE!) ==========
// Mission site target (example values; update from course layout or ask TF):
volatile float TARGET_MISSION_X = 3.00f;  // meters
volatile float TARGET_MISSION_Y = 1.00f;  // meters
// Pre-limbo/log staging point:
volatile float TARGET_LIMBO_X   = 3.40f;  // meters
volatile float TARGET_LIMBO_Y   = 0.50f;  // meters

// ========== STATE MACHINE ==========
enum class RunState : uint8_t {
  WAIT_VISION = 0,
  CALIBRATE_SENSORS,
  FORWARD_DEMO,     // MS5 forward locomotion
  TURNING_DEMO,     // MS5 90° turning
  NAV_TO_MISSION,   // no obstacles required for MS5 #6
  SAMPLE_WATER,     // depth + water type, then mission()
  NAV_TO_LIMBO,     // move to limbo staging
  HOLD              // done
};
RunState g_state = RunState::WAIT_VISION;

// ========== POSE HELPERS ==========
static inline float normalizeAngle(float a) {
  while (a < 0)        a += 2.0f * PI;
  while (a >= 2.0f*PI) a -= 2.0f * PI;
  return a;
}
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline float hypot2(float dx, float dy) {
  return sqrtf(dx*dx + dy*dy);
}
static inline float headingTo(float x0, float y0, float x1, float y1) {
  return atan2f(y1 - y0, x1 - x0);
}

// ========== HARDWARE I/O (replace with your drivers if needed) ==========
static void setMotorPWM(uint8_t pin, int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(pin, pwm);
}
static void stopMotors() {
  setMotorPWM(FL_PWM, 128);
  setMotorPWM(FR_PWM, 128);
  setMotorPWM(BL_PWM, 128);
  setMotorPWM(BR_PWM, 128);
}

// Mecanum kinematics (vx,vy,omega) -> 4 wheel speeds -> PWM
static void driveMecanum(float vx, float vy, float omega) {
  vx = clampf(vx,   -kMaxCmd, kMaxCmd);
  vy = clampf(vy,   -kMaxCmd, kMaxCmd);
  omega = clampf(omega, -kMaxCmd, kMaxCmd);

  float fl = vx - vy - omega;
  float fr = vx + vy + omega;
  float bl = vx + vy - omega;
  float br = vx - vy + omega;

  auto toPWM = [](float s)->int { s = clampf(s, -1.0f, 1.0f); return (int)lroundf(s * 127.0f + 128.0f); };
  setMotorPWM(FL_PWM, toPWM(fl));
  setMotorPWM(FR_PWM, toPWM(fr));
  setMotorPWM(BL_PWM, toPWM(bl));
  setMotorPWM(BR_PWM, toPWM(br));
}

// ========== ENES100 WRAPPERS (wireless RX/TX) ==========
struct Pose {
  float x, y, theta; // meters, meters, radians (-pi..pi per doc)
  bool  visible;
};
static Pose readPoseVision() {
  Pose p;
  p.x = Enes100.getX();
  p.y = Enes100.getY();
  p.theta = Enes100.getTheta();
  p.visible = Enes100.isVisible();
  return p;
}
static void missionPrintln(const String& msg) {
  // Visible on overhead console during dev; for demo use Enes100.mission() to submit results
  Enes100.println(msg);
}

// ========== OBSTACLE SENSING ==========
static float readObstacleRaw() {
  // Example: analog sensor giving higher value when an object is near
  int raw = analogRead(OBST_SENSOR_PIN);
  return (float)raw / 1023.0f * 5.0f; // volts
}
static bool obstacleDetected() {
  // Compare to baseline +/- threshold
  float v = readObstacleRaw();
  return fabsf(v - g_obstBaseline) > g_obstAvoidThresh;
}
static void obstacleAvoidStep() {
  // Simple avoidance: strafe left briefly to go around
  driveMecanum(0.0f, -0.7f, 0.0f);
}

// ========== CALIBRATION ==========
static void calibrateObstacleSensor(uint16_t samples = 100) {
  float acc = 0.0f;
  for (uint16_t i = 0; i < samples; ++i) {
    acc += readObstacleRaw();
    delay(5);
  }
  g_obstBaseline = acc / samples;
  missionPrintln(String("Obstacle baseline V=") + String(g_obstBaseline, 3));
}
static void calibrateDepthZero(uint16_t samples = 100) {
  long acc = 0;
  for (uint16_t i = 0; i < samples; ++i) { acc += analogRead(DEPTH_SENSOR_PIN); delay(5); }
  g_depthZeroOffset = acc / samples;
  missionPrintln(String("Depth zero offset=") + g_depthZeroOffset);
}
static void calibrateTurbidity(uint16_t samples = 100) {
  long acc = 0;
  for (uint16_t i = 0; i < samples; ++i) { acc += analogRead(TURB_SENSOR_PIN); delay(5); }
  int avg = acc / samples;
  // Example: set threshold halfway between avg and a known "dirty" ref if you have it; here we keep avg
  g_turbidityThresh = avg;
  missionPrintln(String("Turbidity thresh set to ") + g_turbidityThresh);
}

// ========== MISSION SENSING / ACTUATION (WATER) ==========
static int readDepthMM() {
  int raw = analogRead(DEPTH_SENSOR_PIN) - g_depthZeroOffset;
  int mm  = (int) lroundf(raw * g_depthScale_mm_per_count);
  return max(0, mm);
}
static bool isPolluted() {
  int raw = analogRead(TURB_SENSOR_PIN);
  return (raw >= g_turbidityThresh); // adjust polarity if your sensor inverts
}
static void sendWaterResults() {
  // Report depth (mm)
  int depth_mm = readDepthMM();
  Enes100.mission(DEPTH, depth_mm);

  // Report water type
  if (isPolluted()) {
    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
    missionPrintln("Mission: WATER_TYPE=FRESH_POLLUTED");
  } else {
    Enes100.mission(WATER_TYPE, FRESH_UNPOLLUTED);
    missionPrintln("Mission: WATER_TYPE=FRESH_UNPOLLUTED");
  }
  missionPrintln(String("Mission: DEPTH=") + depth_mm + " mm");
}

// ========== DEMO MOVES (MS5 #2 Forward, #3 Turning) ==========
static void demoForwardStep() {
  // Simple constant vx for a short time to show locomotion
  driveMecanum(0.6f, 0.0f, 0.0f);
}
static void demoTurn90Step() {
  // Spin in place (omega). Tune to get ~90° within your loop timing or add gyro feedback.
  driveMecanum(0.0f, 0.0f, 0.6f);
}

// ========== NAVIGATION (NO OBSTACLES + AVOIDANCE) ==========
static void translateToward(float x, float y, const Pose& p) {
  float dx = x - p.x;
  float dy = y - p.y;
  float vx = kVTransGain * dx;
  float vy = kVTransGain * dy;
  float mag = hypot2(vx, vy);
  if (mag > kMaxCmd) { vx *= (kMaxCmd / mag); vy *= (kMaxCmd / mag); }
  driveMecanum(vx, vy, 0.0f);
}
static bool arrived(float x, float y, const Pose& p) {
  return hypot2(x - p.x, y - p.y) <= kArriveTol;
}

// ========== SETUP / LOOP ==========
void setup() {
  Serial.begin(9600);

  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
  pinMode(OBST_SENSOR_PIN, INPUT);
  pinMode(DEPTH_SENSOR_PIN, INPUT);
  pinMode(TURB_SENSOR_PIN, INPUT);

  // --- ENES100 begin() blocks until connected to Vision System ---
  Enes100.begin(TEAM_NAME, TEAM_TYPE, MARKER_ID, ROOM_NUMBER, WIFI_TX_PIN, WIFI_RX_PIN);
  // At this point, Enes100.isConnected() == true by contract; we can log:
  missionPrintln("ENES100 connected. WATER mission start.");

  stopMotors();
  g_state = RunState::CALIBRATE_SENSORS;
}

static unsigned long t0 = 0;
static uint8_t turnCount = 0;

void loop() {
  // Optional: safety if vision drops; still allow demos/calibration
  bool visionOK = Enes100.isConnected();

  switch (g_state) {
    case RunState::WAIT_VISION: {
      if (visionOK) {
        missionPrintln("Vision reconnected.");
        g_state = RunState::CALIBRATE_SENSORS;
      }
    } break;

    case RunState::CALIBRATE_SENSORS: {
      calibrateObstacleSensor(120);
      calibrateDepthZero(120);
      calibrateTurbidity(120);
      t0 = millis();
      g_state = RunState::FORWARD_DEMO; // MS5 #2
    } break;

    case RunState::FORWARD_DEMO: {
      // Run forward for ~2 seconds
      demoForwardStep();
      if (millis() - t0 > 2000) {
        stopMotors();
        delay(300);
        t0 = millis();
        turnCount = 0;
        g_state = RunState::TURNING_DEMO; // MS5 #3
      }
    } break;

    case RunState::TURNING_DEMO: {
      // Perform three ~90° turns
      demoTurn90Step();
      if (millis() - t0 > 700) { // tune duration for ~90 deg
        stopMotors();
        delay(300);
        t0 = millis();
        turnCount++;
        if (turnCount >= 3) {
          g_state = RunState::NAV_TO_MISSION; // MS5 #6
        }
      }
    } break;

    case RunState::NAV_TO_MISSION: {
      Pose p = readPoseVision();
      if (!p.visible) { // still okay; dead-reckon if you have odom; here we pause
        missionPrintln("Waiting for vision...");
        stopMotors();
        break;
      }

      // MS5 #7 obstacle avoidance (single obstacle)
      if (obstacleDetected()) {
        missionPrintln("Obstacle detected – avoidance step");
        obstacleAvoidStep();
        break;
      }

      translateToward(TARGET_MISSION_X, TARGET_MISSION_Y, p);
      if (arrived(TARGET_MISSION_X, TARGET_MISSION_Y, p)) {
        stopMotors();
        missionPrintln("Arrived at mission site.");
        g_state = RunState::SAMPLE_WATER;
        t0 = millis();
      }
    } break;

    case RunState::SAMPLE_WATER: {
      // Pause briefly to settle sensors, then send mission() results
      if (millis() - t0 < 500) { stopMotors(); break; }
      sendWaterResults(); // MS5 #9 + #10 for WATER (sensing + actuation via mission())
      missionPrintln("Water results sent.");
      g_state = RunState::NAV_TO_LIMBO; // proceed to limbo/log staging (MS5 #6 continuation)
    } break;

    case RunState::NAV_TO_LIMBO: {
      Pose p = readPoseVision();
      if (!p.visible) { stopMotors(); break; }

      if (obstacleDetected()) {
        missionPrintln("Obstacle detected – avoidance step");
        obstacleAvoidStep();
        break;
      }

      translateToward(TARGET_LIMBO_X, TARGET_LIMBO_Y, p);
      if (arrived(TARGET_LIMBO_X, TARGET_LIMBO_Y, p)) {
        stopMotors();
        missionPrintln("Reached pre-limbo staging point.");
        g_state = RunState::HOLD;
      }
    } break;

    case RunState::HOLD: {
      stopMotors();
      // All MS5 boxes are covered; remain idle
    } break;
  }

  // Watchdog: if vision drops mid-run, we can park or retry
  if (!visionOK && g_state >= RunState::NAV_TO_MISSION) {
    stopMotors();
    g_state = RunState::WAIT_VISION;
  }

  delay(kLoopDtMs);
}