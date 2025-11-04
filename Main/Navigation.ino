// ===== SIMPLEST M5 NAV CODE =====
// Motors ON → delay → stop → delay → motors ON → stop
// No sensing, no feedback. Pure timing.

#define MOTOR_FL_1 2
#define MOTOR_FL_2 3
#define MOTOR_FR_1 4
#define MOTOR_FR_2 5
#define MOTOR_BL_1 6
#define MOTOR_BL_2 7
#define MOTOR_BR_1 8
#define MOTOR_BR_2 9

void setup() {
  // Initialize motor pins
  for (int pin = 2; pin <= 9; pin++) {
    pinMode(pin, OUTPUT);
  }

  delay(2000);   // Small pause before starting
}

void loop() {

  // ========= Go to mission site =========
  driveForward();
  delay(4000);     // adjust time until it reaches mission site
  stopMotors();

  delay(2000);     // pause

  // ========= Go to limbo area =========
  driveForward();
  delay(4000);     // adjust time as needed
  stopMotors();

  while (true);    // done forever
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
  for (int pin = 2; pin <= 9; pin++) {
    digitalWrite(pin, LOW);
  }
}
