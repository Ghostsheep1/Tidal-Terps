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
}
