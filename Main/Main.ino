#include <Enes100.h>

/*
 * ENES100 – MS6 FULL INTEGRATED CODE (CLEANED)
 *  - Manual motor control
 *  - Navigation using Enes100.getX/Y/Theta()
 *  - Mission I: Pollution detection
 *  - Mission II: Water depth measurement
 *  - Mission III: Sample collection (servo/pump)
 */

#include <Arduino.h>
#include "Enes100.h"
#include <Servo.h>

// ===================================================
// TEAM / WIFI CONFIGURATION
// ===================================================
const char* TEAM_NAME   = "Tidal Terps";
const byte  TEAM_TYPE   = WATER;
const int   MARKER_ID   = 0;     
const int   ROOM_NUMBER = 1116;

const int WIFI_TX_PIN = 10;
const int WIFI_RX_PIN = 11;

// ===================================================
// MOTOR CONFIGURATION (Neutral-128 single PWM)
// ===================================================
#define DRIVE_MODE_NEUTRAL 1

const int FL_PWM = 3;
const int FR_PWM = 5;
const int BL_PWM = 6;
const int BR_PWM = 9;

// ===================================================
// MOTOR HELPERS
// ===================================================
int signedToNeutralPWM(float s) {
  s = constrain(s, -1.0, 1.0);
  return int(s * 127.0f + 128.0f);
}

void motorWriteNeutral(float fl, float fr, float bl, float br) {
  analogWrite(FL_PWM, signedToNeutralPWM(fl));
  analogWrite(FR_PWM, signedToNeutralPWM(fr));
  analogWrite(BL_PWM, signedToNeutralPWM(bl));
  analogWrite(BR_PWM, signedToNeutralPWM(br));
}

void setWheelPWM(int leftPercent, int rightPercent) {
  float l = constrain(leftPercent, -100, 100) / 100.0;
  float r = constrain(rightPercent, -100, 100) / 100.0;
  motorWriteNeutral(l, r, l, r);
}

void stopMotors() {
  motorWriteNeutral(0, 0, 0, 0);
}

void driveForward(float s) {
  s = constrain(s, -1, 1);
  motorWriteNeutral(s, s, s, s);
}

void turnLeft(float s) {
  s = constrain(s, -1, 1);
  motorWriteNeutral(-s, s, -s, s);
}

// ===================================================
// MISSION I – POLLUTION DETECTION
// ===================================================
const int  POLLUTANT_SENSOR_PIN = A0;
const bool POLLUTED_ON_HIGH     = false;

bool isWaterPolluted() {
  int v = digitalRead(POLLUTANT_SENSOR_PIN);
  Serial.print("Pollution reading = "); Serial.println(v);
  return POLLUTED_ON_HIGH ? (v == HIGH) : (v == LOW);
}

void missionObjectiveI_PollutionDetection() {
  bool polluted = isWaterPolluted();
  int result = polluted ? FRESH_POLLUTED : FRESH_UNPOLLUTED;

  Serial.print("MISSION_I: ");
  Serial.println(polluted ? "WATER_POLLUTED" : "WATER_CLEAN");

  Enes100.mission(WATER_TYPE, result);
  Enes100.print("MISSION_I_FLAG: ");
  Enes100.println(polluted ? 1 : 0);
}

// ===================================================
// MISSION II – WATER DEPTH MEASUREMENT
// ===================================================
const int DEPTH_TRIG_PIN = 7;
const int DEPTH_ECHO_PIN = 8;

long SENSOR_TO_POOL_BOTTOM_MM = 55;
const unsigned long ULTRA_TIMEOUT = 30000;

long microsecondsToMillimeters(long us) { return (us / 2.9) / 2; }

long measureDistanceToWaterMM() {
  digitalWrite(DEPTH_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(DEPTH_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DEPTH_TRIG_PIN, LOW);

  long duration = pulseIn(DEPTH_ECHO_PIN, HIGH, ULTRA_TIMEOUT);
  if (duration == 0) return -1;

  long mm = microsecondsToMillimeters(duration);
  Serial.print("Surface dist: "); Serial.print(mm); Serial.println(" mm");
  return mm;
}

int getWaterDepthMM() {
  long mm_surface = measureDistanceToWaterMM();
  if (mm_surface < 0) return -1;

  long depth = SENSOR_TO_POOL_BOTTOM_MM - mm_surface;
  if (depth < 0) depth = 0;

  Serial.print("Depth = "); Serial.println(depth);
  return depth;
}

void missionObjectiveII_DepthMeasurement() {
  int depth = getWaterDepthMM();
  if (depth < 0) {
    Serial.println("MISSION_II: DEPTH_ERROR");
    return;
  }

  Serial.print("MISSION_II: DEPTH_MM="); Serial.println(depth);
  Enes100.mission(DEPTH, depth);
}

// ===================================================
// MISSION III – SAMPLE COLLECTION (Servo)
// ===================================================
#define USE_SAMPLE_SERVO

const int SAMPLE_ACTUATOR_PIN = 4;
const int SAMPLE_FULL_SENSOR_PIN = -1;

#ifdef USE_SAMPLE_SERVO
Servo sampleServo;
const int SAMPLE_SERVO_CLOSED = 0;
const int SAMPLE_SERVO_OPEN   = 90;
const unsigned long SAMPLE_OPEN_TIME = 1500;

void openSampleValve() {
  Serial.println("Sample servo: OPEN");
  sampleServo.write(SAMPLE_SERVO_OPEN);
  delay(SAMPLE_OPEN_TIME);
  sampleServo.write(SAMPLE_SERVO_CLOSED);
}
#endif

bool isSampleFull() {
  if (SAMPLE_FULL_SENSOR_PIN < 0) return false;
  return digitalRead(SAMPLE_FULL_SENSOR_PIN) == LOW;
}

void missionObjectiveIII_CollectSample() {
  Serial.println("MISSION_III: START");
  stopMotors();

  if (isSampleFull()) {
    Serial.println("Already full.");
    return;
  }

#ifdef USE_SAMPLE_SERVO
  openSampleValve();
#endif

  Serial.println(isSampleFull() ? "SAMPLE_COLLECTED" : "SAMPLE_NOT_CONFIRMED");
}

// ===================================================
// NAVIGATION
// ===================================================
int stage = 0;
bool missionsDone = false;

double readDistanceSensor(int) { return -1.0; }

void navigationStep() {
  float x = Enes100.getX();
  float y = Enes100.getY();
  float t = Enes100.getTheta();

  // Identify start region
  if (stage == 0) {
    if (abs(y - 0.55) < 0.1) stage = 1; 
    if (abs(y - 1.45) < 0.1) stage = 2;
  }

  // ---- STAGE 1: Move UP ----
  if (stage == 1) {
    if (abs(t - 1.57) > 0.08) {
      setWheelPWM(t < 1.57 ? -50 : 50, t < 1.57 ? 50 : -50);
      delay(10); stopMotors(); delay(10);
      return;
    }
    if (y < 1.42) {
      setWheelPWM(100, 100);
      delay(100); stopMotors(); delay(80);
      return;
    }
    stage = 3;
  }

  // ---- STAGE 2: Move DOWN ----
  if (stage == 2) {
    if (abs(t + 1.57) > 0.08) {
      setWheelPWM(t > -1.57 ? 30 : -30, t > -1.57 ? -30 : 30);
      delay(60); stopMotors(); delay(60);
      return;
    }
    if (y > 0.58) {
      setWheelPWM(70, 70);
      delay(100); stopMotors(); delay(80);
      return;
    }
    stage = 3;
  }

  // ---- Face θ ≈ 0 ----
  while (t <= 0 || t >= 0.1) setWheelPWM(-40, 40);
  setWheelPWM(40, 40);

  // ---- Ensure Y ≥ 1.2 past X ≥ 3.1 ----
  if (x >= 3.1 && y < 1.2) {
    while (abs(t - 1.57) > 0.1)
      setWheelPWM(t < 1.57 ? -40 : 40, t < 1.57 ? 40 : -40);

    while (Enes100.getY() <= 1.2)
      setWheelPWM(70, 70);

    while (t <= 0 || t >= 0.1)
      setWheelPWM(-40, 40);
  }

  // ---- Run Missions ----
  if (!missionsDone && x >= 3.6) {
    stopMotors();

    missionObjectiveI_PollutionDetection();
    missionObjectiveII_DepthMeasurement();
    missionObjectiveIII_CollectSample();

    missionsDone = true;

    while (true) stopMotors();
  }

  // ---- Default forward ----
  setWheelPWM(40, 40);

  Enes100.print("X="); Enes100.println(x);
  Enes100.print("Y="); Enes100.println(y);
  Enes100.print("Theta="); Enes100.println(t);
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Booting...");

  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);

  stopMotors();

  Enes100.begin(TEAM_NAME, TEAM_TYPE, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);

  pinMode(POLLUTANT_SENSOR_PIN, INPUT);

  pinMode(DEPTH_TRIG_PIN, OUTPUT);
  pinMode(DEPTH_ECHO_PIN, INPUT);

  pinMode(SAMPLE_ACTUATOR_PIN, OUTPUT);
  digitalWrite(SAMPLE_ACTUATOR_PIN, LOW);

#ifdef USE_SAMPLE_SERVO
  sampleServo.attach(SAMPLE_ACTUATOR_PIN);
  sampleServo.write(SAMPLE_SERVO_CLOSED);
#endif
}

// ===================================================
// LOOP
// ===================================================
void loop() {
  navigationStep();
}
