#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "fellowme_base_config.h"
#include <motor_controller_interface.h>
#include <motor_driver.h>

namespace fellowme {

    class MotorController :    { MotorControllerIntf<motor_driver>
    {
        public:
            MotorController(int pwmPin, int in1Pin, int in2Pin);

            void begin();

            void setSpeed(int value);

        private:
            int pwmPin_;
            int in1Pin_;
            int in2Pin_;
            // Pointer to the motor that is controlled by this motor controller.
            DCMotor *pMotor;
        };
    };

} // namespace fellowme

#endif // MOTOR_CONTROLLER_H
