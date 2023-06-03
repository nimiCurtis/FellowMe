#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer {
public:
  Timer();
  void start();
  float elapsedSeconds();
private:
  unsigned long startTime;
};

#endif  // TIMER_H
