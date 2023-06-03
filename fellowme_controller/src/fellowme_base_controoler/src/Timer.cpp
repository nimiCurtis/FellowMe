#include "Timer.h"

Timer::Timer() {
  previousTime = 0;
  currentTime = 0;
}

void Timer::start() {
  previousTime = micros();
}

float Timer::elapsedTime() {
  currentTime = micros();
  float elapsedTime = (currentTime - previousTime) / 1000000.0;
  return elapsedTime;
}
