#ifndef L298NCONTROLLER_H
#define L298NCONTROLLER_H

#include <L298N.h>

class L298NController {
public:
  L298NController(int enablePin, int in1Pin, int in2Pin);
  void setSpeed(int pwmValue);

private:
  L298N motor_;
};

#endif
