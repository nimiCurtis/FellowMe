#ifndef DFR0601_MOTOR_CONTROLLER_H
#define DFR0601_MOTOR_CONTROLLER_H

#include "fellowme_base_config.h"
#include <motor_controller_interface.h>
#include <L298N.h>


namespace fellowme {

    /** \brief Implementation of the MotorControllerIntf for the dfr0601
     * 
     * Implements the abstract setSpeed method from the MotorControllerIntf
     * 
     * The class makes use of the L298N library.
     */
    class Dfr0601MotorController :  public MotorControllerIntf<L298N>
    {
        public:
            /** \brief Construct an \ref Dfr0601MotorController for a single motor
             * 
             * Specify the motor pins to control \p enPin , \p in1Pin , \p in2Pin .

             * 
             * \param enPin pin number of pwm channel.
             * \param in1Pin pin number of in1. 
             * \param in2Pin pin 2 of in2.
             */
            Dfr0601MotorController(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin);

            /** \brief Initializes the communication with the motor driver
             * 
             * must be called in setup() to initialize the driver. 
             */
            void begin();


            /** \brief Set the speed of the motor \ref pMotor_.
             * 
             * Concrete implementation of the setSpeed interface method
             * to control a single motor connected to the Dfr0601.
             * The input parameter \p value ranges from -255 to 255.
             * Inside this method the value is mapped between 0 to 255 and 
             * the sign is used to set the direction.
             * A positive \p value results in the motor spinning forward.
             * Negative \p value rotates the motor backward and
             * a zero value stops the motor (releases the current).

             * \param value positive or negative value to set the direction
             * and speed of the motor.
             */
            void setSpeed(int value);

        private:
            // int pwmPin_;
            // int in1Pin_;
            // int in2Pin_;
            // Pointer to the motor that is controlled by this motor controller.
            L298N *pMotor_;
        };

} // namespace fellowme

#endif // DFR0601_MOTOR_CONTROLLER_H
