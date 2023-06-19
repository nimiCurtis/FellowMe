#include "L298NController.h"
#include <L298N.h>

L298NController::L298NController(int enablePin, int in1Pin, int in2Pin)
  : motor_(enablePin, in1Pin, in2Pin) {}
  // {
  //   motor_ = L298N(enablePin, in1Pin, in2Pin);
  // }

void L298NController::setSpeed(int pwmValue) {
  if (pwmValue > 0) {
    motor_.setSpeed(pwmValue);
    motor_.forward();
  } else if (pwmValue < 0) {
    motor_.setSpeed(-pwmValue);
    motor_.backward();
  } else {
    motor_.stop();
  }
}
