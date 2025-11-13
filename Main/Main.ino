/*
 * ENES100 – MS6 NAVIGATION + BASE CODE
 *  - Neutral-128 motor drive
 *  - Navigation logic converted from Tank.* to our own helpers
 *  - ENES100 only (no Tank library)
 */

#include <Arduino.h>
#include "Enes100.h"

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
const int WIFI_TX_PIN = 8;   // TODO: choose allowed TX pin (not 0,1,13 on Uno)
const int WIFI_RX_PIN = 9;   // TODO: choose RX pin

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

// Simple wrappers similar to your earlier helpers (if you still want them)
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
// NAVIGATION STATE
// ===================================================
int stage = 0;

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

  // REACH MISSION REGION
  if (Enes100.getX() >= 3.6) {
    stopMotors();

    // ==========================
    // PLACE TO RUN MISSIONS
    // ==========================
    // TODO: once stopped over the pool, call your mission functions:
    //   missionObjectiveI_PollutionDetection();
    //   missionObjectiveII_DepthMeasurement();
    //   missionObjectiveIII_CollectSample();
    //
    // And send mission() results, e.g.:
    //   Enes100.mission(WATER_TYPE, FRESH_POLLUTED);  // or FRESH_UNPOLLUTED
    //   Enes100.mission(DEPTH, depth_mm);

    while (1) {
      // stay here indefinitely after mission
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

  Enes100.println("MS6 NAVIGATION STARTED");
  Serial.println("ENES100 CONNECTED");

  // TODO: Mission-specific hardware init goes here too, e.g.:
  //   pinMode(POLLUTANT_SENSOR_PIN, INPUT);
  //   pinMode(DEPTH_TRIG_PIN, OUTPUT);
  //   pinMode(DEPTH_ECHO_PIN, INPUT);
  //   etc.
}

// ===================================================
// MAIN LOOP
// ===================================================
void loop() {
  // Navigation runs continuously
  navigationStep();

  // If you want missions to be triggered by Enes100.mission() states,
  // or by stage changes, you can add a state machine here instead of
  // running navigationStep() blindly every loop.
}
