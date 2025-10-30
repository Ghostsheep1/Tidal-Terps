/*
 * Tidal Terps – Minimal Navigation Skeleton (Mecanum OTV)
 * -------------------------------------------------------
 * - Computes a simple direct path (single waypoint) to a destination.
 * - Uses mecanum kinematics (vx, vy, omega) for holonomic motion.
 * - Includes stubs for heading, odometry, and obstacle sensing.
 * - Non-blocking control loop suitable for Arduino loop().
 *
 * Replace the STUB functions with your hardware integrations:
 *   - readHeadingRad(), readPose(), obstacleDetected(), setMotorPWM()
 */

#include <Arduino.h>
#include <math.h>

// ---------- Types ----------
struct Coordinate {
  float x;
  float y;
};

struct Pose2D {
  float x;
  float y;
  float heading; // radians, [0, 2π)
};

// ---------- Config ----------
static const Coordinate kDestination       = { 0.0f, 0.0f };
static const float      kArriveTolMeters   = 0.05f;      // distance tolerance
static const float      kMaxCmd            = 1.0f;       // max magnitude for vx, vy, omega (pre-PWM map)
static const float      kVTransGain        = 0.6f;       // proportional gain for translation
static const float      kVRotGain          = 0.8f;       // proportional gain for rotation (if used)
static const float      kRotateAlignThresh = 5.0f * PI / 180.0f; // 5 degrees
static const uint16_t   kLoopDtMs          = 20;         // control loop period

// Motor PWM pins (update for your wiring)
static const uint8_t FL_PIN = 3;  // Front-left
static const uint8_t FR_PIN = 5;  // Front-right
static const uint8_t BL_PIN = 6;  // Back-left
static const uint8_t BR_PIN = 9;  // Back-right

// ---------- Globals ----------
static Pose2D     g_pose          = {0.0f, 0.0f, 0.0f};
static Coordinate g_path[10];
static int        g_pathLen       = 0;
static int        g_pathIdx       = 0;

// ---------- Utility ----------
static inline float normalizeAngle(float a) {
  while (a < 0)        a += 2.0f * PI;
  while (a >= 2.0f*PI) a -= 2.0f * PI;
  return a;
}

static inline float clamp(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float dist(Coordinate a, Coordinate b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return sqrtf(dx*dx + dy*dy);
}

static inline float headingTo(Coordinate from, Coordinate to) {
  return atan2f(to.y - from.y, to.x - from.x);
}

// ---------- Hardware Stubs (replace with real code) ----------
static float readHeadingRad() {
  // TODO: read from IMU/gyro; must return [0, 2π)
  return g_pose.heading; // placeholder: using internal pose
}

static Pose2D readPose() {
  // TODO: return fused odometry/IMU pose
  return g_pose; // placeholder: using internal pose
}

static bool obstacleDetected() {
  // TODO: read distance sensor(s) and threshold
  return false;
}

static void setMotorPWM(uint8_t pin, int pwm) {
  // TODO: drive your motor controller properly (DIR + PWM, ESC, etc.)
  analogWrite(pin, clamp(pwm, 0, 255));
}

// ---------- Mecanum Drive ----------
static void driveMecanum(float vx, float vy, float omega) {
  // Limit commands
  vx    = clamp(vx,   -kMaxCmd, kMaxCmd);
  vy    = clamp(vy,   -kMaxCmd, kMaxCmd);
  omega = clamp(omega,-kMaxCmd, kMaxCmd);

  // Basic mecanum kinematics (no wheelbase geometry scaling here)
  const float fl = vx - vy - omega;
  const float fr = vx + vy + omega;
  const float bl = vx + vy - omega;
  const float br = vx - vy + omega;

  // Map [-1,1] -> [0,255]
  auto toPWM = [](float s) -> int {
    const float u = clamp(s, -1.0f, 1.0f);
    return (int) lroundf(u * 127.0f + 128.0f);
  };

  const int flPWM = toPWM(fl);
  const int frPWM = toPWM(fr);
  const int blPWM = toPWM(bl);
  const int brPWM = toPWM(br);

  setMotorPWM(FL_PIN, flPWM);
  setMotorPWM(FR_PIN, frPWM);
  setMotorPWM(BL_PIN, blPWM);
  setMotorPWM(BR_PIN, brPWM);

  // Debug
  Serial.print(F("PWM FL/FR/BL/BR: "));
  Serial.print(flPWM); Serial.print(',');
  Serial.print(frPWM); Serial.print(',');
  Serial.print(blPWM); Serial.print(',');
  Serial.println(brPWM);
}

static void stopMotors() {
  setMotorPWM(FL_PIN, 128);
  setMotorPWM(FR_PIN, 128);
  setMotorPWM(BL_PIN, 128);
  setMotorPWM(BR_PIN, 128);
}

// ---------- Navigation ----------
static void computePath(Coordinate start, Coordinate goal) {
  // Direct path (single waypoint). Extend to multi-waypoint if needed.
  g_path[0]  = goal;
  g_pathLen  = 1;
  g_pathIdx  = 0;
}

static void avoidObstacleStep() {
  // Simple evasive step: strafe left briefly.
  // Tune this or swap for a real local planner.
  driveMecanum(0.0f, -0.6f, 0.0f);
}

static void alignHeadingStep(float desiredHeading) {
  const float curr = readHeadingRad();
  const float err  = normalizeAngle(desiredHeading - curr);

  if (fabsf(err) < kRotateAlignThresh) {
    // aligned enough
    driveMecanum(0.0f, 0.0f, 0.0f);
    return;
  }

  const float omegaCmd = clamp(kVRotGain * err, -kMaxCmd, kMaxCmd);
  driveMecanum(0.0f, 0.0f, omegaCmd);
}

static void translateToTargetStep(Coordinate tgt) {
  const Pose2D pose = readPose();

  // P-control on position (world frame). If you want robot-frame control,
  // rotate (dx,dy) by -pose.heading first.
  const float dx = tgt.x - pose.x;
  const float dy = tgt.y - pose.y;

  float vx = kVTransGain * dx;
  float vy = kVTransGain * dy;

  // scale down if exceeding limit
  const float mag = sqrtf(vx*vx + vy*vy);
  if (mag > kMaxCmd) {
    vx *= (kMaxCmd / mag);
    vy *= (kMaxCmd / mag);
  }

  driveMecanum(vx, vy, 0.0f);
}

// Call this each loop() tick; non-blocking.
static void controlLoop() {
  if (g_pathLen == 0 || g_pathIdx >= g_pathLen) {
    stopMotors();
    return;
  }

  // Obstacle handling first
  if (obstacleDetected()) {
    Serial.println(F("Obstacle detected – avoidance step"));
    avoidObstacleStep();
    return;
  }

  // Current target
  const Coordinate tgt = g_path[g_pathIdx];
  const Pose2D     p   = readPose();

  // If you need the robot to face the target before translating, enable align:
  // const float desHead = headingTo({p.x, p.y}, tgt);
  // alignHeadingStep(desHead);
  // if (fabsf(normalizeAngle(desHead - p.heading)) > kRotateAlignThresh) return;

  // Translate toward target
  translateToTargetStep(tgt);

  // Check arrival
  const float d = dist({p.x, p.y}, tgt);
  if (d <= kArriveTolMeters) {
    Serial.print(F("Reached waypoint ")); Serial.println(g_pathIdx);
    ++g_pathIdx;
    if (g_pathIdx >= g_pathLen) {
      Serial.println(F("Path complete."));
      stopMotors();
    }
  }
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(9600);

  pinMode(FL_PIN, OUTPUT);
  pinMode(FR_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(BR_PIN, OUTPUT);

  // Initialize pose if you have a known start
  g_pose = {0.0f, 0.0f, 0.0f};

  // Build the direct path to destination
  computePath({g_pose.x, g_pose.y}, kDestination);

  stopMotors();
}

void loop() {
  controlLoop();

  // Simulated pose update for the placeholders ONLY:
  // Remove this when you hook real odom/IMU.
  // (very rough dead-reckoning towards target)
  // ------------------------------------------------
  if (g_pathIdx < g_pathLen) {
    Coordinate tgt = g_path[g_pathIdx];
    float dx = tgt.x - g_pose.x;
    float dy = tgt.y - g_pose.y;
    float step = 0.005f; // fake meters per tick
    float L = sqrtf(dx*dx + dy*dy);
    if (L > 1e-6f) {
      g_pose.x += step * (dx / L);
      g_pose.y += step * (dy / L);
    }
  }
  // ------------------------------------------------

  delay(kLoopDtMs);
}
