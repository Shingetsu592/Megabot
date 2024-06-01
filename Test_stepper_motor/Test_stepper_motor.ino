// define library
#include <AccelStepper.h>

// define motor stepper pin
#define step_IN1 8
#define step_IN2 9
#define step_IN3 10
#define step_IN4 11

#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, step_IN1, step_IN2, step_IN3, step_IN4);

void setup() {
    // Nothing to do (Stepper Library sets pins as outputs)
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(100);
  stepper.setSpeed(300);
	
}

void step_motor_up(){
  delay(3);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, HIGH);
  delay(3);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, HIGH);
  digitalWrite(step_IN4, LOW);
  delay(3);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, HIGH);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(3);
  digitalWrite(step_IN1, HIGH);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
}

void step_motor_down(){
  // Rotate CW slowly at 3 RPM
  delay(3);
  digitalWrite(step_IN1, HIGH);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(3);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, HIGH);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, LOW);
  delay(3);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, HIGH);
  digitalWrite(step_IN4, LOW);
  delay(3);
  digitalWrite(step_IN1, LOW);
  digitalWrite(step_IN2, LOW);
  digitalWrite(step_IN3, LOW);
  digitalWrite(step_IN4, HIGH);
}

void loop() {
	// Rotate CW slowly at 5 RPM
  // delay(100);
  for (int j = 0; j < 700; j++){
    // step_motor_up();
    step_motor_down();
  }
  for (int i = 0; i < 700; i++){
    // step_motor_down();
    step_motor_up();
  }
  delay(100);
}