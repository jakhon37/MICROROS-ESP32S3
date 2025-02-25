#include "MotorControl.h"

// Initialize static members
volatile int32_t MotorControl::leftEncoderCount = 0;
volatile int32_t MotorControl::rightEncoderCount = 0;
int32_t MotorControl::leftLastCount = 0;
int32_t MotorControl::rightLastCount = 0;
uint32_t MotorControl::lastRPMTime = 0;

void MotorControl::begin() {
  // Configure motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // Configure encoder pins
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // Setup PWM channels using the LEDC API
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_PWM, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_MOTOR_PWM, 1);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  // Initialize RPM timing
  leftLastCount = leftEncoderCount;
  rightLastCount = rightEncoderCount;
  lastRPMTime = millis();
}

void MotorControl::setMotorSpeeds(float leftSpeed, float rightSpeed) {
  setMotorSpeed(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM, leftSpeed);
  setMotorSpeed(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM, rightSpeed);
}

void MotorControl::setMotorSpeed(uint8_t in1, uint8_t in2, uint8_t pwmPin, float speed) {
  uint8_t duty = (uint8_t)(constrain(fabs(speed), 0.0, 1.0) * 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  // Write the PWM duty cycle to the appropriate channel
  if (pwmPin == LEFT_MOTOR_PWM) {
    ledcWrite(0, duty);
  } else if (pwmPin == RIGHT_MOTOR_PWM) {
    ledcWrite(1, duty);
  }
}

void IRAM_ATTR MotorControl::leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B) == HIGH)
    leftEncoderCount--;
  else
    leftEncoderCount++;
}

void IRAM_ATTR MotorControl::rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH)
    rightEncoderCount--;
  else
    rightEncoderCount++;
}

float MotorControl::getLeftRPM() {
  uint32_t currentTime = millis();
  float dt = (currentTime - lastRPMTime) / 1000.0;  // Convert to seconds
  if (dt <= 0) dt = RPM_TIMER_PERIOD / 1000.0;
  int32_t delta = leftEncoderCount - leftLastCount;
  leftLastCount = leftEncoderCount;
  lastRPMTime = currentTime;
  // Calculate RPM: (delta / counts per rev) * (60 / dt)
  return (delta / (float)ENCODER_CPR) * (60.0 / dt);
}

float MotorControl::getRightRPM() {
  uint32_t currentTime = millis();
  float dt = (currentTime - lastRPMTime) / 1000.0;
  if (dt <= 0) dt = RPM_TIMER_PERIOD / 1000.0;
  int32_t delta = rightEncoderCount - rightLastCount;
  rightLastCount = rightEncoderCount;
  lastRPMTime = currentTime;
  return (delta / (float)ENCODER_CPR) * (60.0 / dt);
}
