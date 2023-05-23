#include "motor_controller.h"
#include "motor_driver.h"
#include <Arduino.h>
#include <fellowme_base_config.h>


namespace fellowme {

    MotorController::MotorController(int pwmPin, int in1Pin, int in2Pin)
        : pwmPin_(pwmPin), in1Pin_(in1Pin), in2Pin_(in2Pin)
    {
        pinMode(pwmPin_, OUTPUT);
        pinMode(in1Pin_, OUTPUT);
        pinMode(in2Pin_, OUTPUT);
    }

    void MotorController::begin()
    {
        // No additional initialization required in this case
    }

    void MotorController::setSpeed(int value)
    {
        if (value > 0) {
            digitalWrite(in1Pin_, HIGH);
            digitalWrite(in2Pin_, LOW);
        } else if (value < 0) {
            digitalWrite(in1Pin_, LOW);
            digitalWrite(in2Pin_, HIGH);
            value = -value;
        } else { // value == 0
            digitalWrite(in1Pin_, LOW);
            digitalWrite(in2Pin_, LOW);
        }

        analogWrite(pwmPin_, value);
    }

} // namespace fellowme
