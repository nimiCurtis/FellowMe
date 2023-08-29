#include "L298NController.h"
#include <L298N.h>

L298NController::L298NController(int enablePin, int in1Pin, int in2Pin)
  : motor_(enablePin, in1Pin, in2Pin) {}

void L298NController::setSpeed(int pwmValue) {
  if (pwmValue > 5) {
    
    if (pwmValue > 15){
      motor_.setSpeed(pwmValue);
    }else{
      motor_.setSpeed(10);
    }
  motor_.forward();
  } else if (pwmValue < -5) {
    // make it positive
    if (pwmValue < -15){
      motor_.setSpeed(-pwmValue);
    }else{
      motor_.setSpeed(10);
    }
    motor_.backward();
  } else {
    motor_.stop();
  }
}
