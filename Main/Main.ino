/*******************************************************************
 ENES100 + Tank Demo
 Navigation: Random Start → Mission Site → Pre-Limbo Waypoint
 Uses ALL Tank movement + sensor functions
*******************************************************************/

#include "Tank.h"
#include "Enes100.h"

// ------------------------
// UPDATE THESE
// ------------------------
const char TEAM_NAME[] = "TEAM_NAME";
const int ARUCO_ID = 0;
const Enes100::Material MATERIAL = Enes100::Material::WOOD;

// Example coordinates — REPLACE WITH REAL ONES
float X_MS  = 1.0;   // mission site
float Y_MS  = 2.0;
float X_LIM = 3.0;   // before limbo/log
float Y_LIM = 3.2;

float posX, posY, heading;

// motion settings
int fwdPWM  = 140;
int turnPWM = 150;

// --------------------------------------------
// Basic helpers
// --------------------------------------------

void stopAll() {
  Tank.turnOffMotors();
}

// drive forward
void driveForward() {
  Tank.setLeftMotorPWM(fwdPWM);
  Tank.setRightMotorPWM(fwdPWM);
}

// drive backward
void driveBackward() {
  Tank.setLeftMotorPWM(-fwdPWM);
  Tank.setRightMotorPWM(-fwdPWM);
}

// turn in place left
void rotateLeft() {
  Tank.setLeftMotorPWM(-turnPWM);
  Tank.setRightMotorPWM(turnPWM);
}

// turn in place right
void rotateRight() {
  Tank.setLeftMotorPWM(turnPWM);
  Tank.setRightMotorPWM(-turnPWM);
}

// directly set individual motor (1-4)
void motorExample() {
  Tank.setMotorPWM(1, 150);   // forward motor 1
  Tank.setMotorPWM(2, 150);   // forward motor 2
  delay(300);
  Tank.turnOffMotors();
}

// update pose from vision
void updatePose() {
  posX    = Enes100.getX();
  posY    = Enes100.getY();
  heading = Enes100.getTheta();
}

// wrap angle
float wrapAngle(float a) {
  while (a > 3.14159)  a -= 2 * 3.14159;
  while (a < -3.14159) a += 2 * 3.14159;
  return a;
}

// turn to angle target
void rotateTo(float targetTheta) {
  updatePose();
  float error = wrapAngle(targetTheta - heading);

  while (abs(error) > 0.12) {
    if (error > 0) rotateLeft();
    else           rotateRight();

    delay(50);
    updatePose();
    error = wrapAngle(targetTheta - heading);
  }

  stopAll();
}

// move to coordinate
void goTo(float targetX, float targetY) {
  updatePose();

  float dx = targetX - posX;
  float dy = targetY - posY;
  float dist = sqrt(dx*dx + dy*dy);
  float targetHeading = atan2(dy, dx);

  // face target
  rotateTo(targetHeading);

  // drive forward
  while (dist > 0.12) {
    driveForward();
    delay(80);

    updatePose();
    dx = targetX - posX;
    dy = targetY - posY;
    dist = sqrt(dx*dx + dy*dy);
  }
  stopAll();
}

// -----------------------------------------------------
// Setup
// -----------------------------------------------------
void setup() {
  Serial.begin(9600);
  Tank.begin();

  Enes100.begin(TEAM_NAME, MATERIAL, ARUCO_ID, 1116, 52, 50);
  delay(300);

  Enes100.println("START Navigation");

  // read sensors: demo usage
  float dist1 = Tank.readDistanceSensor(1);
  int ir = Tank.readIrSensor(1);
  int bump = Tank.readBumpSensor(1);

  Enes100.print("Ultrasonic: ");
  Enes100.println(dist1);

  Enes100.print("IR1: ");
  Enes100.println(ir);

  Enes100.print("Bump front: ");
  Enes100.println(bump);

  // example: control individual motors
  motorExample();
}

// -----------------------------------------------------
// Main Loop
// -----------------------------------------------------
void loop() {

  updatePose();
  Enes100.print("Start at (");
  Enes100.print(posX);
  Enes100.print(",");
  Enes100.print(posY);
  Enes100.println(")");

  //-------------------------------------------
  // Go to Mission Site
  //-------------------------------------------
  Enes100.println("Going to mission site...");
  goTo(X_MS, Y_MS);

  Enes100.println("At mission site.");
  delay(800);

  //-------------------------------------------
  // Go to Pre-Limbo Point
  //-------------------------------------------
  Enes100.println("Going to limbo start...");
  goTo(X_LIM, Y_LIM);

  Enes100.println("Arrived at limbo staging.");

  stopAll();

  // hold nicely
  while (true);
}
