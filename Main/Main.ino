/*
 * Navigation tasks for mecanum-wheel OTV.
 * Implements path computation, coordinate/heading usage, and control loop.
 * This code is designed for a vehicle with four mecanum wheels (four motors),
 * enabling holonomic movement (forward/backward, lateral, and rotation).
 */

#include <Arduino.h>
#include <math.h>

// Structure representing 2D coordinates
struct Coordinate {
  float x;
  float y;
};

// Global variables for destination and current position
Coordinate destination = {0.0, 0.0};
Coordinate currentPosition = {0.0, 0.0};

// Path array and length (simple direct path for demonstration)
Coordinate path[10];
int pathLength = 0;

// Motor control pins for the 4 mecanum wheels (placeholders – update with real pins)
const int FL_PIN = 3; // Front-left motor PWM pin
const int FR_PIN = 5; // Front-right motor PWM pin
const int BL_PIN = 6; // Back-left motor PWM pin
const int BR_PIN = 9; // Back-right motor PWM pin

// Compute a simple path from start to end (could be expanded with waypoints)
void computePath(Coordinate start, Coordinate end) {
  // For now, just set a direct path to the end point
  path[0] = end;
  pathLength = 1;
}

// Compute the heading (radians) from current to target
float computeHeading(Coordinate current, Coordinate target) {
  float dx = target.x - current.x;
  float dy = target.y - current.y;
  return atan2(dy, dx);
}

// Normalize heading to [0, 2*pi)
float normalizeAngle(float angle) {
  while (angle < 0) angle += 2 * PI;
  while (angle >= 2 * PI) angle -= 2 * PI;
  return angle;
}

// Drive the mecanum wheels given desired velocities along x (forward/back), y (strafe), and rotational velocity omega
void driveMecanum(float vx, float vy, float omega) {
  // Compute wheel speeds using basic mecanum kinematics
  float fl = vx - vy - omega;
  float fr = vx + vy + omega;
  float bl = vx + vy - omega;
  float br = vx - vy + omega;

  // Scale and constrain values to PWM range (0–255) for demonstration
  int flPWM = constrain((int)(fl * 128 + 128), 0, 255);
  int frPWM = constrain((int)(fr * 128 + 128), 0, 255);
  int blPWM = constrain((int)(bl * 128 + 128), 0, 255);
  int brPWM = constrain((int)(br * 128 + 128), 0, 255);

  // Write to motors (replace analogWrite with actual motor driver control if necessary)
  analogWrite(FL_PIN, flPWM);
  analogWrite(FR_PIN, frPWM);
  analogWrite(BL_PIN, blPWM);
  analogWrite(BR_PIN, brPWM);

  // Debug output
  Serial.print("Wheel PWM: ");
  Serial.print(flPWM); Serial.print(", ");
  Serial.print(frPWM); Serial.print(", ");
  Serial.print(blPWM); Serial.print(", ");
  Serial.println(brPWM);
}

// Rotate to the desired heading by commanding a rotational velocity
void rotateToHeading(float desiredHeading) {
  // Placeholder: in a real system, read the current heading from a gyro or IMU
  float currentHeading = 0.0;
  float error = normalizeAngle(desiredHeading - currentHeading);

  // Simple proportional control to rotate the robot; adjust gain as needed
  float omega = 0.5 * error;
  driveMecanum(0.0, 0.0, omega);
}

// Move towards a target coordinate using mecanum motion (translation along x and y)
void moveToTarget(Coordinate target) {
  float dx = target.x - currentPosition.x;
  float dy = target.y - currentPosition.y;

  // Compute simple proportional velocities (tuning gains as needed)
  float vx = 0.2 * dx;
  float vy = 0.2 * dy;

  // No rotation while translating
  driveMecanum(vx, vy, 0.0);
}

// Control loop to follow the computed path
void controlLoop() {
  if (pathLength == 0) {
    return;
  }

  // Iterate through each waypoint in the path
  for (int i = 0; i < pathLength; i++) {
    Coordinate target = path[i];

    // Compute the desired heading and rotate to face the target
    float heading = computeHeading(currentPosition, target);
    rotateToHeading(heading);

    // Move towards the target; in a real system, loop until close enough
    moveToTarget(target);

    // Update currentPosition (placeholder; in practice, update from odometry/IMU)
    currentPosition = target;
  }
}

void setup() {
  Serial.begin(9600);
  // Initialize motor pins as outputs
  pinMode(FL_PIN, OUTPUT);
  pinMode(FR_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(BR_PIN, OUTPUT);

  // Compute the initial path
  computePath(currentPosition, destination);
}

void loop() {
  // Execute control loop to follow the path
  controlLoop();

  // Delay for demonstration; in a real system, use a smaller delay or none
  delay(1000);
}
