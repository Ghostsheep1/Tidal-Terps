/*
 * ENES100 – MS6 FULL INTEGRATED CODE
 *  - Navigation (no Tank library)
 *  - Mission I: Pollution detection (digital sensor)
 *  - Mission II: Water depth (ultrasonic, trig=7 echo=8)
 *  - Mission III: Sample collection (servo or pump)
 */

#include <Arduino.h>
#include "Enes100.h"
#include <Servo.h>

// ===================================================
// REQUIRED ENES100 SETTINGS  (EDIT THESE)
// ===================================================
const char* TEAM_NAME   = "Tidal Terps";
const byte  TEAM_TYPE   = WATER;
const int   MARKER_ID   = 0;      // TODO: set your ArUco marker ID
const int   ROOM_NUMBER = 1116;

// Wi-Fi module pins (EDIT based on wiring)
// Connect Arduino pin WIFI_TX_PIN → WiFi TX
// Connect Arduino pin WIFI_RX_PIN → WiFi RX
// On Uno, TX pin cannot be 0,1,13; we also avoid motor/sensor pins.
const int WIFI_TX_PIN = 10;   // TODO: choose allowed TX pin
const int WIFI_RX_PIN = 11;   // TODO: choose RX pin

// ===================================================
//   MOTOR WIRING MODE — NEUTRAL-128 SINGLE PWM
// ===================================================
#define DRIVE_MODE_NEUTRAL 1
const int FL_PWM = 3;
const int FR_PWM = 5;
const int BL_PWM = 6;
const int BR_PWM = 9;
// DIR pins NOT used in this mode

// ===================================================
// BASIC MOTOR HELPERS (NEUTRAL-128 MODE)
// ===================================================
#ifdef DRIVE_MODE_NEUTRAL
// Map -1..1 to 0..255, where 128 is stop
int signedToNeutralPWM(float s) {
  s = constrain(s, -1.0f, 1.0f);
  return (int)(s * 127.0f + 128.0f);
}

// Low-level function: set each wheel to a signed speed -1..1
void motorWriteNeutral(float fl, float fr, float bl, float br) {
  analogWrite(FL_PWM, signedToNeutralPWM(fl));
  analogWrite(FR_PWM, signedToNeutralPWM(fr));
  analogWrite(BL_PWM, signedToNeutralPWM(bl));
  analogWrite(BR_PWM, signedToNeutralPWM(br));
}
#endif

// ===================================================
// HIGH-LEVEL DRIVE HELPERS (replacing Tank.*)
// ===================================================

// Set “tank style” left/right speeds in percent (-100..100)
void setWheelPWM(int leftPercent, int rightPercent) {
  float l = constrain(leftPercent, -100, 100) / 100.0f;
  float r = constrain(rightPercent, -100, 100) / 100.0f;
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(l, r, l, r);
#endif
}

void stopMotors() {
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(0, 0, 0, 0);
#endif
}

// Simple wrappers if you want direct forward/turn functions
void driveForward(float speed) { // speed -1..1
  speed = constrain(speed, -1, 1);
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(speed, speed, speed, speed);
#endif
}

void turnLeft(float speed) { // speed -1..1
  speed = constrain(speed, -1, 1);
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(-speed, speed, -speed, speed);
#endif
}

// ===================================================
// MISSION-SPECIFIC OBJECTIVE I: POLLUTION DETECTION
// ===================================================

// Digital pollution sensor pin (e.g., IR, limit switch, etc.)
const int POLLUTANT_SENSOR_PIN = A0;   // TODO: change to your actual pin
const bool POLLUTED_ON_HIGH    = false; // true if HIGH = polluted, false if LOW = polluted

// Read the pollution sensor and return true if water is polluted.
bool isWaterPolluted() {
  int val = digitalRead(POLLUTANT_SENSOR_PIN);
  Serial.print("Pollution digital reading = ");
  Serial.println(val);

  bool polluted;
  if (POLLUTED_ON_HIGH) {
    polluted = (val == HIGH);
  } else {
    polluted = (val == LOW);
  }
  return polluted;
}

// Mission I:
// 1) Decide if water is polluted
// 2) Transmit via print + Enes100.mission(WATER_TYPE, ...)
void missionObjectiveI_PollutionDetection() {
  bool polluted = isWaterPolluted();

  if (polluted) {
    Enes100.println("MISSION_I: WATER_POLLUTED");
    Serial.println("MISSION_I: WATER_POLLUTED");
    // Send mission result as WATER_TYPE
    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
  } else {
    Enes100.println("MISSION_I: WATER_CLEAN");
    Serial.println("MISSION_I: WATER_CLEAN");
    Enes100.mission(WATER_TYPE, FRESH_UNPOLLUTED);
  }

  // Optional compact flag
  Enes100.print("MISSION_I_FLAG:");
  Enes100.println(polluted ? 1 : 0);
}

// ===================================================
// MISSION-SPECIFIC OBJECTIVE II: WATER DEPTH (ULTRASONIC)
// ===================================================

// Using your calibration wiring: trigPin = 7, echoPin = 8
const int DEPTH_TRIG_PIN = 7;
const int DEPTH_ECHO_PIN = 8;

// Vertical distance from sensor face to pool bottom (cm)
// TODO: measure with a ruler when robot is in mission position
const float SENSOR_TO_POOL_BOTTOM_CM = 7.0;

// Optional timeout for pulseIn to avoid hanging
const unsigned long ULTRASONIC_TIMEOUT_US = 30000; // 30 ms

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING)))
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // Speed of sound is ~29 µs per cm, round trip
  return microseconds / 29 / 2;
}

// Measure distance from sensor to water surface in centimeters.
// Returns negative value if no valid reading.
float measureDistanceToWaterCM() {
  // Trigger pulse
  digitalWrite(DEPTH_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(DEPTH_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DEPTH_TRIG_PIN, LOW);

  // Echo
  unsigned long duration = pulseIn(DEPTH_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    Serial.println("Ultrasonic timeout: no echo");
    return -1.0;
  }

  long cm = microsecondsToCentimeters(duration);

  Serial.print("Sensor → water surface: ");
  Serial.print(cm);
  Serial.println(" cm");

  return (float)cm;
}

// Returns estimated water depth in mm. Negative value = error.
int getWaterDepthMM() {
  float d_surface_cm = measureDistanceToWaterCM();
  if (d_surface_cm < 0) {
    return -1;
  }

  float depth_cm = SENSOR_TO_POOL_BOTTOM_CM - d_surface_cm;
  if (depth_cm < 0) depth_cm = 0;

  int depth_mm = (int)(depth_cm * 10.0 + 0.5); // cm -> mm

  Serial.print("Computed water depth: ");
  Serial.print(depth_cm);
  Serial.print(" cm (");
  Serial.print(depth_mm);
  Serial.println(" mm)");

  return depth_mm;
}

// Mission II:
// 1) Measure water depth in mm
// 2) Transmit via print + Enes100.mission(DEPTH, depth_mm)
void missionObjectiveII_DepthMeasurement() {
  int depth_mm = getWaterDepthMM();

  if (depth_mm < 0) {
    Enes100.println("MISSION_II: DEPTH_ERROR");
    Serial.println("MISSION_II: DEPTH_ERROR");
    return;
  }

  Enes100.print("MISSION_II: DEPTH_MM=");
  Enes100.println(depth_mm);
  Serial.print("MISSION_II: DEPTH_MM=");
  Serial.println(depth_mm);

  // Enes100 mission call for depth (mm)
  Enes100.mission(DEPTH, depth_mm);

  // Optional compact message
  Enes100.print("MISSION_II_DEPTH_ONLY:");
  Enes100.println(depth_mm);
}

// ===================================================
// MISSION-SPECIFIC OBJECTIVE III: SAMPLE COLLECTION
// ===================================================

// Choose ONE of these and comment out the other:
//#define USE_SAMPLE_PUMP
#define USE_SAMPLE_SERVO   // default: using servo mechanism

// Pick a free digital pin NOT already used.
// Avoid 3,5,6,9 (motors) and 7,8 (ultrasonic).
const int SAMPLE_ACTUATOR_PIN = 4;  // TODO: change if pin 4 is not free

// Optional "sample full" sensor (e.g., microswitch). -1 = no sensor used.
const int SAMPLE_FULL_SENSOR_PIN = -1; // TODO: set to real pin if you have one

// ---- PUMP VERSION SETTINGS ----
#ifdef USE_SAMPLE_PUMP
// How long to run pump for ~20 mL (ms). Tune this!
const unsigned long SAMPLE_PUMP_FILL_TIME_MS = 2500;  // TODO: adjust
const int SAMPLE_PUMP_POWER = 255; // 0–255
#endif

// ---- SERVO VERSION SETTINGS ----
#ifdef USE_SAMPLE_SERVO
Servo sampleServo;

// Servo angles for your mechanism
const int SAMPLE_SERVO_CLOSED_ANGLE = 0;   // TODO: set closed position
const int SAMPLE_SERVO_OPEN_ANGLE   = 90;  // TODO: set open/collect position

// How long to keep servo open to collect ~20 mL (ms). Tune this!
const unsigned long SAMPLE_SERVO_OPEN_TIME_MS = 1500;  // TODO: adjust
#endif

// Returns true if "sample full" sensor says the sample is already collected.
// If SAMPLE_FULL_SENSOR_PIN == -1, always returns false.
bool isSampleAlreadyFull() {
  if (SAMPLE_FULL_SENSOR_PIN < 0) {
    return false;  // no sensor configured
  }

  int val = digitalRead(SAMPLE_FULL_SENSOR_PIN);
  // Example: active-low microswitch (pressed = LOW = full)
  bool full = (val == LOW);

  Serial.print("Sample full sensor reading: ");
  Serial.println(val);
  Serial.print("Sample considered full: ");
  Serial.println(full ? "YES" : "NO");

  return full;
}

// ---- PUMP VERSION ----
#ifdef USE_SAMPLE_PUMP
void runSamplePump(unsigned long runTimeMs) {
  Serial.println("Sample pump: ON");
  analogWrite(SAMPLE_ACTUATOR_PIN, SAMPLE_PUMP_POWER);
  delay(runTimeMs);
  analogWrite(SAMPLE_ACTUATOR_PIN, 0);
  Serial.println("Sample pump: OFF");
}
#endif

// ---- SERVO VERSION ----
#ifdef USE_SAMPLE_SERVO
void openSampleValve(unsigned long openTimeMs) {
  Serial.println("Sample servo: OPEN");
  sampleServo.write(SAMPLE_SERVO_OPEN_ANGLE);
  delay(openTimeMs);
  Serial.println("Sample servo: CLOSE");
  sampleServo.write(SAMPLE_SERVO_CLOSED_ANGLE);
}
#endif

// Mission III: collect and retain ~20 mL sample
void missionObjectiveIII_CollectSample() {
  Serial.println("MISSION_III: Starting sample collection");
  Enes100.println("MISSION_III: START");

  // Ensure robot isn’t moving
  stopMotors();

  // If sensor says already full, skip
  if (isSampleAlreadyFull()) {
    Serial.println("MISSION_III: Sample already full, no actuation needed.");
    Enes100.println("MISSION_III: SAMPLE_ALREADY_FULL");
    return;
  }

  // Actuate mechanism
#ifdef USE_SAMPLE_PUMP
  runSamplePump(SAMPLE_PUMP_FILL_TIME_MS);
#endif

#ifdef USE_SAMPLE_SERVO
  openSampleValve(SAMPLE_SERVO_OPEN_TIME_MS);
#endif

  bool fullNow = isSampleAlreadyFull();

  if (fullNow || SAMPLE_FULL_SENSOR_PIN < 0) {
    Enes100.println("MISSION_III: SAMPLE_COLLECTED");
    Serial.println("MISSION_III: SAMPLE_COLLECTED");
  } else {
    Enes100.println("MISSION_III: SAMPLE_NOT_CONFIRMED");
    Serial.println("MISSION_III: SAMPLE_NOT_CONFIRMED");
  }
}

// ===================================================
// NAVIGATION STATE
// ===================================================
int stage = 0;
bool missionsDone = false;

// TODO: implement your actual distance sensor.
// For now, this stub returns -1 (meaning “no reading/no obstacle”).
double readDistanceSensor(int /*id*/) {
  // Hook up your real sensor here and return distance in meters
  // or -1 if no object detected.
  return -1.0;
}

// This function is your old navigation logic with all Tank.* calls
// replaced by setWheelPWM() + stopMotors(), and using Enes100 for pose.
void navigationStep() {
  float x     = Enes100.getX();
  float y     = Enes100.getY();
  float theta = Enes100.getTheta();

  // DECIDE WHICH WAY FIRST
  if (stage == 0) {
    if (abs(y - 0.55) < 0.1) stage = 1;   // start bottom -> go UP
    if (abs(y - 1.45) < 0.1) stage = 2;   // start top -> go DOWN
  }

  // STAGE 1: GO UP TO y ≈ 1.45
  if (stage == 1) {

    // align to +1.57 (up)
    if (abs(theta - 1.57) > 0.08) {
      if (theta < 1.57) {
        setWheelPWM(-50, 50);
      } else {
        setWheelPWM(50, -50);
      }
      delay(10);
      stopMotors();
      delay(10);
      return;
    }

    // drive up until y ≥ 1.40
    if (y < 1.42) {
      setWheelPWM(100, 100);
      delay(100);
      stopMotors();
      delay(80);
      return;
    }

    // reached top
    stage = 3;   // now go to limbo / next phase
  }

  // STAGE 2: GO DOWN TO y ≈ 0.55
  if (stage == 2) {

    // align to -1.57 (down)
    if (abs(theta + 1.57) > 0.08) {
      if (theta > -1.57) {
        setWheelPWM(30, -30);
      } else {
        setWheelPWM(-30, 30);
      }
      delay(60);
      stopMotors();
      delay(60);
      return;
    }

    // drive down until y ≤ 0.55
    if (y > 0.58) {
      setWheelPWM(70, 70);
      delay(100);
      stopMotors();
      delay(80);
      return;
    }
    stage = 3;
  }

  // ============================
  // STRAIGHT + ADJUSTMENT LOGIC
  // ============================

  // Rotate to face roughly θ ≈ 0
  while (Enes100.getTheta() <= 0 || Enes100.getTheta() >= 0.1) {
    setWheelPWM(-40, 40);
  }
  setWheelPWM(40, 40);

  // --- Make sure OTV is above y = 1.2 when x >= 3.1 ---
  if (Enes100.getX() >= 3.1 && Enes100.getY() < 1.2) {

    // Turn to face upward (+1.57 radians)
    while (abs(Enes100.getTheta() - 1.57) > 0.1) {
      if (Enes100.getTheta() < 1.57) {
        setWheelPWM(-40, 40);
      } else {
        setWheelPWM(40, -40);
      }
      delay(50);
      stopMotors();
      delay(50);
    }

    // Move up until y > 1.2
    while (Enes100.getY() <= 1.2) {
      setWheelPWM(70, 70);
      delay(100);
      stopMotors();
      delay(80);
    }

    // Reorient back to face forward (θ ≈ 0)
    if (abs(theta - 1.57) > 0.08) {
      if (theta < 1.57) {
        setWheelPWM(-50, 50);
      } else {
        setWheelPWM(50, -50);
      }
      delay(10);
      stopMotors();
      delay(10);
      return;
    }
  }
  // --- END ADDITION ---

  // REACH MISSION REGION (e.g., near the pool)
  if (!missionsDone && Enes100.getX() >= 3.6) {
    stopMotors();

    // ==========================
    // RUN ALL THREE MISSIONS
    // ==========================
    missionObjectiveI_PollutionDetection();
    missionObjectiveII_DepthMeasurement();
    missionObjectiveIII_CollectSample();

    missionsDone = true;

    // After missions completed, you can either stay put or
    // add code to drive to the finish. For now, loop forever.
    while (1) {
      // hold position
      stopMotors();
    }
  }

  // OBSTACLE AVOIDANCE REGION (using your distance sensor)
  double distance = readDistanceSensor(1);
  Enes100.println(distance);

  while (distance != -1 && distance <= 0.3) {
    while (Enes100.getTheta() <= 1.5) {
      setWheelPWM(-40, 40);
    }
    if (Enes100.getY() >= 1.7) {
      while (Enes100.getTheta() >= -1.4 || Enes100.getTheta() <= -1.7) {
        setWheelPWM(-40, 40);
      }
      setWheelPWM(40, 40);
      delay(8000);
      while (Enes100.getTheta() <= 0 || Enes100.getTheta() >= 0.1) {
        setWheelPWM(-40, 40);
      }
    } else {
      setWheelPWM(40, 0);  // approximate “left motor only” behavior
      delay(2500);
      while (Enes100.getTheta() >= 0) {
        setWheelPWM(40, -40);
      }
    }
    distance = readDistanceSensor(1);
  }

  // Default: keep moving forward
  setWheelPWM(40, 40);

  // Debug prints
  Enes100.print("X = ");     Enes100.println(Enes100.getX());
  Enes100.print("Y = ");     Enes100.println(Enes100.getY());
  Enes100.print("Theta = "); Enes100.println(Enes100.getTheta());
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  // Motor pin modes
  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);

  stopMotors();

  // ENES100 Wi-Fi init
  Enes100.begin(TEAM_NAME, TEAM_TYPE, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);

  Enes100.println("MS6 FULL INTEGRATED STARTED");
  Serial.println("ENES100 CONNECTED");

  // Mission I sensor init
  pinMode(POLLUTANT_SENSOR_PIN, INPUT);

  // Mission II depth sensor init
  pinMode(DEPTH_TRIG_PIN, OUTPUT);
  pinMode(DEPTH_ECHO_PIN, INPUT);
  digitalWrite(DEPTH_TRIG_PIN, LOW);

  // Mission III sample actuator init
  pinMode(SAMPLE_ACTUATOR_PIN, OUTPUT);
  digitalWrite(SAMPLE_ACTUATOR_PIN, LOW);

  if (SAMPLE_FULL_SENSOR_PIN >= 0) {
    pinMode(SAMPLE_FULL_SENSOR_PIN, INPUT_PULLUP); // or INPUT depending on wiring
  }

#ifdef USE_SAMPLE_SERVO
  sampleServo.attach(SAMPLE_ACTUATOR_PIN);
  sampleServo.write(SAMPLE_SERVO_CLOSED_ANGLE);
#endif
}

// ===================================================
// MAIN LOOP
// ===================================================
void loop() {
  // Navigation runs continuously, and when we reach the mission region,
  // navigationStep() will call all three mission objectives once.
  navigationStep();
}
