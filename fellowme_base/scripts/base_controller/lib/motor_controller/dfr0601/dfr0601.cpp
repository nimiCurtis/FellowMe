//#include "dfr0601.h"
#include "dfr0601/dfr0601.h"

#include <Arduino.h>
#include "fellowme_base_config.h"
#include <L298N.h>
//include "L298N.h"


fellowme::Dfr0601MotorController::Dfr0601MotorController(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin)
    // : enPin_(enPin), in1Pin_(in1Pin), in2Pin_(in2Pin)
{   
    motor_driver_ = L298N(in1Pin, in2Pin, enPin);
    
}

void fellowme::Dfr0601MotorController::begin()
{
    //nothiong to initialize in this case
}

void fellowme::Dfr0601MotorController::setSpeed(int value)
{
    if (value > 0)
    {
        motor_driver_.run(L298N::FORWARD);
    }
    else if (value < 0)
    {
        motor_driver_.run(L298N::BACKWARD);
        // AdafruAdafruit_MotorShield requires uint8 values from 0 to 255
        // Make sure to convert the negative value to a positive one
        value = value * -1;
    }
    else // zero speed
    {
        // Cut power to the motor
        motor_driver_.run(L298N::STOP);
    }

    motor_driver_.setSpeed(value);
}
