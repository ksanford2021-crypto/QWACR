// Simple open-loop motor test: spins each motor in turn.

const int MOTOR_FL_PWM = 9;
const int MOTOR_FL_DIR = 8;

const int MOTOR_BL_PWM = 10;
const int MOTOR_BL_DIR = 7;

const int MOTOR_FR_PWM = 11;
const int MOTOR_FR_DIR = 6;

const int MOTOR_BR_PWM = 12;
const int MOTOR_BR_DIR = 5;

const int DRIVER_1_M1SLP = 37;
const int DRIVER_1_M2SLP = 29;
const int DRIVER_2_M1SLP = 30;
const int DRIVER_2_M2SLP = 31;

void setup() {
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FL_DIR, OUTPUT);
  pinMode(MOTOR_BL_PWM, OUTPUT);
  pinMode(MOTOR_BL_DIR, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT);
  pinMode(MOTOR_FR_DIR, OUTPUT);
  pinMode(MOTOR_BR_PWM, OUTPUT);
  pinMode(MOTOR_BR_DIR, OUTPUT);

  pinMode(DRIVER_1_M1SLP, OUTPUT);
  pinMode(DRIVER_1_M2SLP, OUTPUT);
  pinMode(DRIVER_2_M1SLP, OUTPUT);
  pinMode(DRIVER_2_M2SLP, OUTPUT);

  // Wake all drivers
  digitalWrite(DRIVER_1_M1SLP, HIGH);
  digitalWrite(DRIVER_1_M2SLP, HIGH);
  digitalWrite(DRIVER_2_M1SLP, HIGH);
  digitalWrite(DRIVER_2_M2SLP, HIGH);
}

void stopAll() {
  analogWrite(MOTOR_FL_PWM, 0);
  analogWrite(MOTOR_BL_PWM, 0);
  analogWrite(MOTOR_FR_PWM, 0);
  analogWrite(MOTOR_BR_PWM, 0);
}

void loop() {
  int pwm = 120;  // adjust as needed

  // FL
  stopAll();
  digitalWrite(MOTOR_FL_DIR, HIGH);
  analogWrite(MOTOR_FL_PWM, pwm);
  delay(3000);

  // BL
  stopAll();
  digitalWrite(MOTOR_BL_DIR, HIGH);
  analogWrite(MOTOR_BL_PWM, pwm);
  delay(3000);

  // FR
  stopAll();
  digitalWrite(MOTOR_FR_DIR, HIGH);
  analogWrite(MOTOR_FR_PWM, pwm);
  delay(3000);

  // BR
  stopAll();
  digitalWrite(MOTOR_BR_DIR, HIGH);
  analogWrite(MOTOR_BR_PWM, pwm);
  delay(3000);

  stopAll();
  delay(3000);
}
