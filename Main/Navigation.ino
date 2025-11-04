#include <ENES100.h>
#include <math.h>

// Motor pins
#define MOTOR_FL_1 2
#define MOTOR_FL_2 3
#define MOTOR_FR_1 4
#define MOTOR_FR_2 5
#define MOTOR_BL_1 6
#define MOTOR_BL_2 7
#define MOTOR_BR_1 8
#define MOTOR_BR_2 9

void setup() {
  Serial.begin(9600);

  Enes100.begin("TidalTerp", WATER, 534, 1116, 8, 9);

  while (!Enes100.isConnected()) {
    Enes100.println("Waiting for connection...");
    delay(1000);
  }

  Enes100.println("Connected – Begin Mission!");

  // Initialize motors
  for (int pin = 2; pin <= 9; pin++) pinMode(pin, OUTPUT);
}

void loop() {

  float x = Enes100.getX();
  float y = Enes100.getY();
  float theta = Enes100.getTheta();

  // Example coordinates — replace with mission site & limbo positions
  float missionX = 2.0;
  float missionY = 1.5;
  float limboX = 3.2;
  float limboY = 1.5;

  // Move to mission
  moveTo(missionX, missionY);

  stopMotors();
  delay(2000);

  // Move to limbo
  moveTo(limboX, limboY);

  stopMotors();
  Enes100.println("Navigation Complete!");

  while (true); // stop forever
}

void moveTo(float targetX, float targetY) {

  float startX = Enes100.getX();
  float startY = Enes100.getY();

  float distance = sqrt(pow(targetX - startX, 2) + pow(targetY - startY, 2));

  Enes100.println("Moving to target...");

  // Basic forward only — no turning correction
  driveForward();

  delay(distance * 1000);   // approx. 1 sec per meter
}

void driveForward() {
  digitalWrite(MOTOR_FL_1, HIGH);
  digitalWrite(MOTOR_FL_2, LOW);
  digitalWrite(MOTOR_FR_1, HIGH);
  digitalWrite(MOTOR_FR_2, LOW);
  digitalWrite(MOTOR_BL_1, HIGH);
  digitalWrite(MOTOR_BL_2, LOW);
  digitalWrite(MOTOR_BR_1, HIGH);
  digitalWrite(MOTOR_BR_2, LOW);
}

void stopMotors() {
  for (int pin = 2; pin <= 9; pin++) digitalWrite(pin, LOW);
}
