/*
 * Navigation code for no obstacles and obstacle avoidance.
 *
 * This file implements basic navigation logic for the Tidal Terps robot.
 * It computes heading from current coordinates to target coordinates,
 * rotates the robot to the desired heading, moves forward to the target,
 * and optionally handles obstacle avoidance using a distance sensor.
 *
 * These functions are simple stubs and should be integrated with actual motor control
 * and sensor reading code.
 * Navigation tasks for mecanum-wheel OTV.
 * Implements path computation, coordinate/heading usage, and control loop.
 * This code is designed for a vehicle with four mecanum wheels (four motors),
 * enabling holonomic movement (forward/backward, lateral, and rotation).
 */

#include <Arduino.h>
#include <math.h>

// Define a structure for a 2D coordinate
struct Coordinate {
  float x;
  float y;
};

// Example target destination; this could be updated dynamically
Coordinate destination = {0.0, 0.0};

// Function prototypes
float computeHeading(Coordinate current, Coordinate target);
float normalizeAngle(float angle);
void rotateToHeading(float desiredHeading);
void moveForward(float distance);
bool obstacleDetected();
void avoidObstacle();
void navigateToPoint(Coordinate target);

void setup() {
  // initialize motors, sensors, and serial communication
  Serial.begin(9600);
  // Additional setup code can go here
}

void loop() {
  // In a real application, current position would be determined via localization
  Coordinate current = {0.0, 0.0};

  // Attempt to navigate to the destination
  navigateToPoint(destination);

  // Delay to avoid spamming commands; adjust as needed
  delay(1000);
}

// Compute the heading (radians) from current position to the target position
float computeHeading(Coordinate current, Coordinate target) {
  float dx = target.x - current.x;
  float dy = target.y - current.y;
  float heading = atan2(dy, dx); // result in radians
  return heading;
}

// Normalize an angle to the range [0, 2*PI)
float normalizeAngle(float angle) {
  while (angle < 0) {
    angle += 2 * PI;
  }
  while (angle >= 2 * PI) {
    angle -= 2 * PI;
  }
  return angle;
}

// Rotate the robot to a desired heading
void rotateToHeading(float desiredHeading) {
  // Placeholder: read the current heading from a gyro/compass sensor
  float currentHeading = 0.0;
  float error = normalizeAngle(desiredHeading - currentHeading);

  // Send commands to motors to rotate until the error is minimized
  // This stub only prints information; replace with actual motor control
  Serial.print("Rotating to heading (radians): ");
  Serial.println(desiredHeading);
  // TODO: implement motor control here
}

// Move the robot forward by a specified distance (in whatever units are appropriate)
void moveForward(float distance) {
  // Placeholder: command motors to move forward by the given distance
  Serial.print("Moving forward distance: ");
  Serial.println(distance);
  // TODO: implement motor control here
}

// Check if an obstacle is detected using a distance sensor
bool obstacleDetected() {
  // Placeholder: return true if an obstacle is within a threshold distance
  // Replace with actual sensor reading code, e.g., using an ultrasonic sensor
  return false;
}

// Define behaviour to avoid an obstacle
void avoidObstacle() {
  // Simple obstacle avoidance: rotate 90 degrees and move a bit forward
  Serial.println("Obstacle detected! Executing avoidance manoeuvre...");
  rotateToHeading(PI / 2); // rotate 90 degrees
  moveForward(10.0);       // move forward an arbitrary distance
}

// Navigate towards a target point, handling obstacles if necessary
void navigateToPoint(Coordinate target) {
  // If an obstacle is detected, perform obstacle avoidance and return early
  if (obstacleDetected()) {
    avoidObstacle();
    return;
  }

  // Determine the desired heading from current position to the target
  Coordinate current = {0.0, 0.0}; // In real code, update this with current position
  float desiredHeading = computeHeading(current, target);

  // Rotate to face the target
  rotateToHeading(desiredHeading);

  // Compute distance to the target
  float dx = target.x - current.x;
  float dy = target.y - current.y;
  float distance = sqrt(dx * dx + dy * dy);

  // Move forward toward the target
  moveForward(distance);
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
