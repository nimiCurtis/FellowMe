#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <L298N.h>

class MotorController {
public:
  MotorController(unsigned int enPin, unsigned int in1Pin, unsigned int in2Pin);
  void setSpeed(int speed);
  void forward();
  void backward();
  void stop();
private:
  L298N motor;
};

#endif  // MOTOR_CONTROLLER_H
