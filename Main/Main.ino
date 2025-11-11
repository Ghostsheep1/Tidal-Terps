/*
 * ENES100 – MS5 BASIC DEMO
 * SUBTASKS SUPPORTED:
 *  #2 Forward Locomotion
 *  #3 Turning (90° x3)
 *  #4 Wireless Receive
 *  #5 Wireless Transmit
 *
 *  ======== EDIT THESE IF YOU KNOW THEM ========
 */

#include <Arduino.h>
#include "Enes100.h"

// ===================================================
// REQUIRED ENES100 SETTINGS  (EDIT THESE)
// ===================================================
const char* TEAM_NAME   = "Tidal Terps";
const byte  TEAM_TYPE   = WATER;
const int   MARKER_ID   = 0;                  // <-- EDIT
const int   ROOM_NUMBER = 1116;

// Wi-Fi module pins (EDIT based on wiring)
const int WIFI_TX_PIN = 0;   // Arduino → WiFi RX EDIT
const int WIFI_RX_PIN = 0;   // Arduino ← WiFi TX EDIT

// ===================================================
//   MOTOR WIRING MODE — CHOOSE ONE
//   A) DIR + PWM  (L298N style)
//   B) Neutral-128 single PWM
// ===================================================

// ======= A) DIR + PWM MODE =======
// Uncomment if USING DIR + PWM driver
/*
#define DRIVE_MODE_DIR  1
// FRONT LEFT
const int FL_PWM = 3;
const int FL_DIR = 2;

// FRONT RIGHT
const int FR_PWM = 5;
const int FR_DIR = 12;

// BACK LEFT
const int BL_PWM = 6;
const int BL_DIR = A3;

// BACK RIGHT
const int BR_PWM = 9;
const int BR_DIR = A4;
*/

// ======= B) NEUTRAL-128 SINGLE PWM MODE =======
// Uncomment if NO DIR PINS
#define DRIVE_MODE_NEUTRAL 1
const int FL_PWM = 3;
const int FR_PWM = 5;
const int BL_PWM = 6;
const int BR_PWM = 9;
// DIR pins NOT used in this mode


// ===================================================
// BASIC MOTOR HELPERS
// ===================================================

// ---------- DIR + PWM MODE ----------
//Uncomment if USING DIR + PWM MODE
/*
#ifdef DRIVE_MODE_DIR
void motorWriteSigned(float fl, float fr, float bl, float br) {
  auto out = [](int pwmPin, int dirPin, float s) {
    bool dir = (s >= 0.0f);
    int m = (int)round(abs(s) * 255);
    digitalWrite(dirPin, dir ? HIGH : LOW);
    analogWrite(pwmPin, m);
  };
  out(FL_PWM, FL_DIR, fl);
  out(FR_PWM, FR_DIR, fr);
  out(BL_PWM, BL_DIR, bl);
  out(BR_PWM, BR_DIR, br);
}
#endif


// ---------- NEUTRAL-128 MODE ----------
#ifdef DRIVE_MODE_NEUTRAL
void motorWriteNeutral(float fl, float fr, float bl, float br) {
  auto toPWM = [](float s) {
    s = constrain(s, -1, 1);
    return (int)(s * 127.0f + 128.0f);   // 128 = stop
  };
  analogWrite(FL_PWM, toPWM(fl));
  analogWrite(FR_PWM, toPWM(fr));
  analogWrite(BL_PWM, toPWM(bl));
  analogWrite(BR_PWM, toPWM(br));
}
#endif
*/

// ===================================================
// MOVEMENT HELPERS (both modes call these wrappers)
// ===================================================
void stopMotors() {
#ifdef DRIVE_MODE_DIR
  motorWriteSigned(0, 0, 0, 0);
#endif
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(0, 0, 0, 0);
#endif
}


// forward (all wheels same direction)
void driveForward(float speed) {
  speed = constrain(speed, -1, 1);
#ifdef DRIVE_MODE_DIR
  motorWriteSigned(speed, speed, speed, speed);
#endif
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(speed, speed, speed, speed);
#endif
}


// turn left (right wheels fwd, left wheels rev)
void turnLeft(float speed) {
  speed = constrain(speed, -1, 1);
#ifdef DRIVE_MODE_DIR
  motorWriteSigned(-speed, speed, -speed, speed);
#endif
#ifdef DRIVE_MODE_NEUTRAL
  motorWriteNeutral(-speed, speed, -speed, speed);
#endif
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

#ifdef DRIVE_MODE_DIR
  pinMode(FL_DIR, OUTPUT);
  pinMode(FR_DIR, OUTPUT);
  pinMode(BL_DIR, OUTPUT);
  pinMode(BR_DIR, OUTPUT);
#endif

  stopMotors();

  // --- ENES100 Wi-Fi init ---
  Enes100.begin(TEAM_NAME, TEAM_TYPE, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);

  Enes100.println("MS5 STARTED");
  Serial.println("ENES100 CONNECTED");
}


// ===================================================
// SIMPLE TEST STATE MACHINE FOR MS5
// ===================================================
int step = 0;
unsigned long t0 = 0;

void loop() {

}
