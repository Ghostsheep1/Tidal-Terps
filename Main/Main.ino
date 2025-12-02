/*
 * ENES100 - MS6 WATER MISSION - INTEGRATED CODE
 * Team: Tidal Terps
 * Board: Arduino Mega 2560
 * 
 * Mission Objectives:
 *  I.   Water Type Detection (Color Sensor - TCS3200)
 *  II.  Water Depth Measurement (Ultrasonic - Bottom)
 *  III. Sample Collection (Servo + Vacuum Pump)
 */

#include <Arduino.h>
#include "Enes100.h"
#include <Servo.h>

// ===================================================
// TEAM CONFIGURATION
// ===================================================
const char* TEAM_NAME   = "Tidal Terps";
const byte  TEAM_TYPE   = WATER;      
const int   MARKER_ID   = 512;       
const int   ROOM_NUMBER = 1201;     

// ===================================================
// PIN ASSIGNMENTS - ARDUINO MEGA
// ===================================================

// Wi-Fi Module (Hardware Serial1)
const int WIFI_TX_PIN = 50;  // TX1 on Mega
const int WIFI_RX_PIN = 51;  // RX1 on Mega

// Motor Shield Pins
const int MOTOR_M1_PIN1 = 11;
const int MOTOR_M1_PIN2 = 12;
const int MOTOR_M2_PIN1 = 3;
const int MOTOR_M2_PIN2 = 4;
const int MOTOR_M3_PIN1 = 5;
const int MOTOR_M3_PIN2 = 7;
const int MOTOR_M4_PIN1 = 8;
const int MOTOR_M4_PIN2 = 9;

// Color Sensor (TCS3200) - Mission I
const int COLOR_S0  = 28;
const int COLOR_S1  = 29;
const int COLOR_S2  = 30;
const int COLOR_S3  = 31;
const int COLOR_OUT = 32;

// Ultrasonic Sensors
const int US_FRONT_TRIG = 24;
const int US_FRONT_ECHO = 25;
const int US_BOTTOM_TRIG = 26;  // For depth measurement
const int US_BOTTOM_ECHO = 27;

// Sample Collection Mechanism - Mission III
const int SERVO_PIN = 44;
const int PUMP_PIN  = 46;

// ===================================================
// MOTOR CONTROL
// ===================================================

// Motor configuration: M1=FL, M2=FR, M3=BL, M4=BR
void setupMotors() {
  pinMode(MOTOR_M1_PIN1, OUTPUT);
  pinMode(MOTOR_M1_PIN2, OUTPUT);
  pinMode(MOTOR_M2_PIN1, OUTPUT);
  pinMode(MOTOR_M2_PIN2, OUTPUT);
  pinMode(MOTOR_M3_PIN1, OUTPUT);
  pinMode(MOTOR_M3_PIN2, OUTPUT);
  pinMode(MOTOR_M4_PIN1, OUTPUT);
  pinMode(MOTOR_M4_PIN2, OUTPUT);
}

void setMotorSpeed(int m1Speed, int m2Speed, int m3Speed, int m4Speed) {
  // M1 - Front Left
  if (m1Speed > 0) {
    analogWrite(MOTOR_M1_PIN1, m1Speed);
    analogWrite(MOTOR_M1_PIN2, 0);
  } else {
    analogWrite(MOTOR_M1_PIN1, 0);
    analogWrite(MOTOR_M1_PIN2, -m1Speed);
  }
  
  // M2 - Front Right
  if (m2Speed > 0) {
    analogWrite(MOTOR_M2_PIN1, m2Speed);
    analogWrite(MOTOR_M2_PIN2, 0);
  } else {
    analogWrite(MOTOR_M2_PIN1, 0);
    analogWrite(MOTOR_M2_PIN2, -m2Speed);
  }
  
  // M3 - Back Left
  if (m3Speed > 0) {
    analogWrite(MOTOR_M3_PIN1, m3Speed);
    analogWrite(MOTOR_M3_PIN2, 0);
  } else {
    analogWrite(MOTOR_M3_PIN1, 0);
    analogWrite(MOTOR_M3_PIN2, -m3Speed);
  }
  
  // M4 - Back Right
  if (m4Speed > 0) {
    analogWrite(MOTOR_M4_PIN1, m4Speed);
    analogWrite(MOTOR_M4_PIN2, 0);
  } else {
    analogWrite(MOTOR_M4_PIN1, 0);
    analogWrite(MOTOR_M4_PIN2, -m4Speed);
  }
}

void driveForward(int speed) {
  speed = constrain(speed, 0, 255);
  setMotorSpeed(speed, speed, speed, speed);
}

void driveBackward(int speed) {
  speed = constrain(speed, 0, 255);
  setMotorSpeed(-speed, -speed, -speed, -speed);
}

void turnLeft(int speed) {
  speed = constrain(speed, 0, 255);
  setMotorSpeed(-speed, speed, -speed, speed);
}

void turnRight(int speed) {
  speed = constrain(speed, 0, 255);
  setMotorSpeed(speed, -speed, speed, -speed);
}

void stopMotors() {
  setMotorSpeed(0, 0, 0, 0);
}

// ===================================================
// MISSION I: WATER TYPE DETECTION (COLOR SENSOR)
// ===================================================

void setupColorSensor() {
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, LOW);
}

// Read color frequency for a specific filter
long readColorFrequency(char color) {
  switch(color) {
    case 'R':  // Red
      digitalWrite(COLOR_S2, LOW);
      digitalWrite(COLOR_S3, LOW);
      break;
    case 'G':  // Green
      digitalWrite(COLOR_S2, HIGH);
      digitalWrite(COLOR_S3, HIGH);
      break;
    case 'B':  // Blue
      digitalWrite(COLOR_S2, LOW);
      digitalWrite(COLOR_S3, HIGH);
      break;
    case 'C':  // Clear (no filter)
      digitalWrite(COLOR_S2, HIGH);
      digitalWrite(COLOR_S3, LOW);
      break;
  }
  
  delay(100);  // Allow sensor to settle
  return pulseIn(COLOR_OUT, LOW, 50000);  // 50ms timeout
}

bool isWaterPolluted() {
  long red   = readColorFrequency('R');
  long green = readColorFrequency('G');
  long blue  = readColorFrequency('B');
  
  Serial.print("RGB: ");
  Serial.print(red);
  Serial.print(", ");
  Serial.print(green);
  Serial.print(", ");
  Serial.println(blue);
  
  // Determine if polluted based on color readings
  // Adjust these thresholds based on calibration
  // Lower frequency = more light detected
  
  // Example logic: if green is significantly lower than red/blue, water is polluted
  bool polluted = (green < red * 0.7) && (green < blue * 0.7);
  
  return polluted;
}

void missionObjectiveI_WaterType() {
  Serial.println("MISSION I: Detecting Water Type...");
  Enes100.println("MISSION_I: START");
  
  stopMotors();
  delay(500);  // Allow readings to stabilize
  
  bool polluted = isWaterPolluted();
  
  if (polluted) {
    Serial.println("MISSION_I: FRESH_POLLUTED");
    Enes100.println("MISSION_I: FRESH_POLLUTED");
    Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
  } else {
    Serial.println("MISSION_I: FRESH_UNPOLLUTED");
    Enes100.println("MISSION_I: FRESH_UNPOLLUTED");
    Enes100.mission(WATER_TYPE, FRESH_UNPOLLUTED);
  }
  
  delay(500);
}

// ===================================================
// MISSION II: WATER DEPTH MEASUREMENT
// ===================================================

void setupDepthSensor() {
  pinMode(US_BOTTOM_TRIG, OUTPUT);
  pinMode(US_BOTTOM_ECHO, INPUT);
  digitalWrite(US_BOTTOM_TRIG, LOW);
}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  
  if (duration == 0) {
    return -1;  // No echo received
  }
  
  // Convert to millimeters (speed of sound = 343 m/s)
  // distance = (duration / 2) / 29.1 cm = (duration / 2) / 2.91 mm
  long mm = (duration * 10) / 58;  // Simplified: duration / 2.9 / 2
  
  return mm;
}

int getWaterDepthMM() {
  // Distance from sensor to pool bottom when empty (calibrate this!)
  const long SENSOR_TO_BOTTOM_MM = 150;  // Adjust based on your setup
  
  long distanceToSurface = measureDistance(US_BOTTOM_TRIG, US_BOTTOM_ECHO);
  
  if (distanceToSurface < 0) {
    Serial.println("Depth sensor error: no echo");
    return -1;
  }
  
  Serial.print("Distance to surface: ");
  Serial.print(distanceToSurface);
  Serial.println(" mm");
  
  long depth = SENSOR_TO_BOTTOM_MM - distanceToSurface;
  if (depth < 0) depth = 0;
  
  int d20 = abs(depth-20), d30 = abs(depth-30), d40 = abs(depth-40);
  return (d20<d30 && d20<d40)?20:(d30<d20 && d30<d40)?30:40;

}

void missionObjectiveII_WaterDepth() {
  Serial.println("MISSION II: Measuring Water Depth...");
  Enes100.println("MISSION_II: START");
  
  stopMotors();
  delay(500);
  
  int depth = getWaterDepthMM();
  
  if (depth < 0) {
    Serial.println("MISSION_II: ERROR");
    Enes100.println("MISSION_II: DEPTH_ERROR");
    return;
  }
  
  Serial.print("MISSION_II: DEPTH = ");
  Serial.print(depth);
  Serial.println(" mm");
  
  Enes100.print("MISSION_II: DEPTH_MM=");
  Enes100.println(depth);
  
  Enes100.mission(DEPTH, depth);
  
  delay(500);
}

// ===================================================
// MISSION III: SAMPLE COLLECTION
// ===================================================

Servo sampleServo;

void setupSampleCollection() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  
  sampleServo.attach(SERVO_PIN);
  sampleServo.write(0);  // Closed position
}

void collectSample() {
  const int SERVO_OPEN_ANGLE = 90;   // Adjust as needed
  const int SERVO_CLOSE_ANGLE = 0;
  const unsigned long COLLECTION_TIME_MS = 3000;  // 3 seconds
  
  Serial.println("Opening collection mechanism...");
  sampleServo.write(SERVO_OPEN_ANGLE);
  delay(500);
  
  Serial.println("Starting vacuum pump...");
  digitalWrite(PUMP_PIN, HIGH);
  delay(COLLECTION_TIME_MS);
  
  Serial.println("Stopping pump...");
  digitalWrite(PUMP_PIN, LOW);
  delay(500);
  
  Serial.println("Closing collection mechanism...");
  sampleServo.write(SERVO_CLOSE_ANGLE);
  delay(500);
}

void missionObjectiveIII_SampleCollection() {
  Serial.println("MISSION III: Collecting Sample...");
  Enes100.println("MISSION_III: START");
  
  stopMotors();
  delay(1000);
  
  collectSample();
  
  Serial.println("MISSION_III: SAMPLE_COLLECTED");
  Enes100.println("MISSION_III: COMPLETE");
  
  delay(500);
}

// ===================================================
// NAVIGATION
// ===================================================

int navigationStage = 0;
bool missionsCompleted = false;

void alignToAngle(float targetAngle, float tolerance = 0.1) {
  while (abs(Enes100.getTheta() - targetAngle) > tolerance) {
    float currentAngle = Enes100.getTheta();
    float diff = targetAngle - currentAngle;
    
    // Handle angle wrap-around
    if (diff > PI) diff -= 2*PI;
    if (diff < -PI) diff += 2*PI;
    
    if (diff > 0) {
      turnLeft(60);
    } else {
      turnRight(60);
    }
    
    delay(50);
    stopMotors();
    delay(50);
  }
  stopMotors();
}

void navigate() {
  float x = Enes100.getX();
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
  
  // Debug output
  Serial.print("Position: X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" Theta=");
  Serial.println(theta);
  
  // Stage 0: Determine initial direction
  if (navigationStage == 0) {
    if (y < 1.0) {
      navigationStage = 1;  // Start low, go up
      Serial.println("Starting from bottom - going UP");
    } else {
      navigationStage = 2;  // Start high, go down
      Serial.println("Starting from top - going DOWN");
    }
    return;
  }
  
  // Stage 1: Move up to y ≈ 1.4
  if (navigationStage == 1) {
    if (y < 1.35) {
      alignToAngle(PI/2, 0.15);  // Point up (90 degrees)
      driveForward(120);
      delay(200);
      stopMotors();
      delay(100);
    } else {
      Serial.println("Reached top corridor");
      navigationStage = 3;
    }
    return;
  }
  
  // Stage 2: Move down to y ≈ 0.6
  if (navigationStage == 2) {
    if (y > 0.65) {
      alignToAngle(-PI/2, 0.15);  // Point down (-90 degrees)
      driveForward(120);
      delay(200);
      stopMotors();
      delay(100);
    } else {
      Serial.println("Reached bottom corridor");
      navigationStage = 3;
    }
    return;
  }
  
  // Stage 3: Drive forward to mission site
  if (navigationStage == 3) {
    // Align to face forward (0 radians)
    alignToAngle(0, 0.1);
    
    // Check if we need to adjust Y position for obstacles
    if (x > 3.0 && y < 1.1) {
      Serial.println("Adjusting Y position for safety");
      alignToAngle(PI/2, 0.15);
      while (Enes100.getY() < 1.2) {
        driveForward(100);
        delay(100);
        stopMotors();
        delay(50);
      }
    }
    
    // Drive forward to mission area
    if (x < 3.5 && !missionsCompleted) {
      driveForward(100);
      delay(150);
      stopMotors();
      delay(100);
    } else if (!missionsCompleted) {
      navigationStage = 4;  // Reached mission site
    }
    return;
  }
  
  // Stage 4: Execute missions
  if (navigationStage == 4 && !missionsCompleted) {
    stopMotors();
    Serial.println("\n=== EXECUTING ALL MISSIONS ===\n");
    Enes100.println("ARRIVED AT MISSION SITE");
    
    delay(1000);
    
    missionObjectiveI_WaterType();
    delay(1000);
    
    missionObjectiveII_WaterDepth();
    delay(1000);
    
    missionObjectiveIII_SampleCollection();
    delay(1000);
    
    missionsCompleted = true;
    Serial.println("\n=== ALL MISSIONS COMPLETE ===\n");
    Enes100.println("ALL MISSIONS COMPLETE");
    
    navigationStage = 5;
    return;
  }
  
  // Stage 5: Hold position after missions
  if (navigationStage == 5) {
    stopMotors();
    delay(1000);
  }
}

// ===================================================
// OBSTACLE AVOIDANCE (FRONT ULTRASONIC)
// ===================================================

float getFrontDistance() {
  long mm = measureDistance(US_FRONT_TRIG, US_FRONT_ECHO);
  if (mm < 0) return -1;
  return mm / 1000.0;  // Convert to meters
}

// ===================================================
// SETUP
// ===================================================

void setup() {
  Serial.begin(9600);
  Serial.println("\n\n=== ENES100 WATER MISSION ===");
  Serial.println("Team: Tidal Terps");
  Serial.println("Initializing...\n");
  
  // Initialize motors
  setupMotors();
  stopMotors();
  
  // Initialize Wi-Fi and Vision System
  Serial.println("Connecting to Vision System...");
  Enes100.begin(TEAM_NAME, TEAM_TYPE, MARKER_ID, ROOM_NUMBER,
                WIFI_TX_PIN, WIFI_RX_PIN);
  
  Serial.println("Connected to Vision System!");
  Enes100.println("=== SYSTEM ONLINE ===");
  
  // Initialize sensors
  setupColorSensor();
  setupDepthSensor();
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  
  // Initialize sample collection
  setupSampleCollection();
  
  Serial.println("All systems ready!\n");
  delay(2000);
}

// ===================================================
// MAIN LOOP
// ===================================================

void loop() {
  // Check if we can get position data
  if (!Enes100.isVisible()) {
    Serial.println("Marker not visible - waiting...");
    stopMotors();
    delay(500);
    return;
  }
  
  // Execute navigation
  navigate();
  
  delay(50);  // Small delay for stability
}
