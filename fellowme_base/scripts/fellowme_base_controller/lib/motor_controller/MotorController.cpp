#include "MotorController.h"

MotorController::MotorController(int enablePin, int in1Pin, int in2Pin) : motor(enablePin, in1Pin, in2Pin) {
  requestedPWM = 0;
}


void MotorController::setPWM(int pwm) {
  requestedPWM = pwm;
  if (pwm > 0) {
    motor.setSpeed(pwm);
    motor.forward();
  } else if (pwm < 0) {
    motor.setSpeed(-pwm);
    motor.backward();
  } else {
    motor.stop();
  }
}

int MotorController::getRequestedPWM() {
  return requestedPWM;
}
