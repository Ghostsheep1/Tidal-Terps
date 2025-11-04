/*******************************************************************
 ENES100 + Tank Navigation Demo
 (No obstacles — Navigation only)

 Demonstrates basic navigation from:
   - Random starting pose 
   - To mission site (X_MS, Y_MS)
   - Then to pre-limbo waypoint (X_LIM, Y_LIM)

 Requires:
   Tank.h
   Enes100.h

 Change:
   - TEAM_NAME
   - MATERIAL
   - ARUCO_ID
   - Mission & Limbo coordinates
*******************************************************************/

#include "Tank.h"
#include "Enes100.h"

// ======== USER-UPDATE VALUES ========
const char TEAM_NAME[] = "TEAM_NAME_HERE";
const int ARUCO_ID = 0;   // <- Insert yours
const Enes100::Material MATERIAL = Enes100::Material::WOOD; // or FOAM, METAL, etc.

// Mission site location (example only)
float X_MS = 1.0;     // meters
float Y_MS = 2.5;     // meters

// Pre-limbo point (just before log/limbo)
float X_LIM = 3.0;    // meters
float Y_LIM = 3.0;    // meters

// Drive parameters
float forwardPWM = 150;
float turnPWM    = 150;
float posX, posY, heading;

// ======= Simple helper functions =======

// Stop
void stopMotion() {
  Tank.turnOffMotors();
}

// Drive straight forward
void driveForward(int pwm) {
  Tank.setLeftMotorPWM(pwm);
  Tank.setRightMotorPWM(pwm);
}

// Turn left (in place)
void turnLeft(int pwm) {
  Tank.setLeftMotorPWM(-pwm);
  Tank.setRightMotorPWM(pwm);
}

// Turn right (in place)
void turnRight(int pwm) {
  Tank.setLeftMotorPWM(pwm);
  Tank.setRightMotorPWM(-pwm);
}

// Update local pose
void updatePose() {
  posX     = Enes100.getX();
  posY     = Enes100.getY();
  heading  = Enes100.getTheta();   // radians
}

// Convert angle to ±π
float wrapAngle(float a){
  while(a >  3.14159) a -= 2*3.14159;
  while(a < -3.14159) a += 2*3.14159;
  return a;
}

// Rotate to a target angle (in radians)
void rotateTo(float targetTheta) {
  updatePose();
  float error = wrapAngle(targetTheta - heading);

  while (abs(error) > 0.1) {
    if (error > 0) turnLeft(turnPWM);
    else           turnRight(turnPWM);

    delay(50);
    updatePose();
    error = wrapAngle(targetTheta - heading);
  }

  stopMotion();
}

// Drive to a coordinate
void goTo(float targetX, float targetY) {

  updatePose();

  float dx = targetX - posX;
  float dy = targetY - posY;
  float distance = sqrt(dx*dx + dy*dy);
  float targetHeading = atan2(dy, dx);

  // Step 1: Face target
  rotateTo(targetHeading);

  // Step 2: Drive straight
  while (distance > 0.10) {  // Stop within 10 cm
    driveForward(forwardPWM);
    delay(80);

    updatePose();
    dx = targetX - posX;
    dy = targetY - posY;
    distance = sqrt(dx*dx + dy*dy);
  }

  stopMotion();
}

// ======= MAIN =======

void setup() {
  Serial.begin(9600);
  Tank.begin();

  // Connect to vision system
  // ENES100.begin(name, material, arucoID, teamNumber, TX, RX)
  Enes100.begin(TEAM_NAME, MATERIAL, ARUCO_ID, 1116, 52, 50);
  delay(300);

  Enes100.println("Navigation demo start");
}

void loop() {

  // Get initial pose
  updatePose();
  Enes100.print("Starting pose: (");
  Enes100.print(posX);
  Enes100.print(", ");
  Enes100.print(posY);
  Enes100.print(") Heading: ");
  Enes100.println(heading);

  /***********************************************
    STEP 1 — Go to Mission Site
  ***********************************************/
  Enes100.println("Heading to Mission Site...");
  goTo(X_MS, Y_MS);

  Enes100.println("Arrived at Mission Site");

  delay(1000);

  /***********************************************
    STEP 2 — Go to Pre-Limbo Point
  ***********************************************/
  Enes100.println("Heading to Pre-Limbo Waypoint...");
  goTo(X_LIM, Y_LIM);

  Enes100.println("Arrived at Pre-Limbo Waypoint");

  /***********************************************
    DONE
  ***********************************************/
  stopMotion();
  Enes100.println("Navigation complete — Waiting.");

  while(true);   // Hold state
}
