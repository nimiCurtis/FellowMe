#include "Arduino.h"
#include "pid.h"

fellowme::PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
}

double fellowme::PID::compute(float setpoint, float measured_value, float dt)
{
    double error;
    double pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    proportional_ = error;
    integral_ += error*dt;
    derivative_ = (error - prev_error_)/dt;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }

    pid = (kp_ * proportional_) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}

void fellowme::PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}