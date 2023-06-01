/// Encoder pins
#define ENCODER_LEFT_H1 21
#define ENCODER_LEFT_H2 20

#define ENCODER_RIGHT_H1 18
#define ENCODER_RIGHT_H2 17

// Encoder resolution used for initialization 
// will be read from parameter server
#define ENCODER_RESOLUTION 560 // or 140??


// /// Motor i2c address
// #define MOTOR_DRIVER_ADDR 0x60
// #define MOTOR_LEFT 4
// #define MOTOR_RIGHT 3

// Motor Driver pins
#define MOTOR_LEFT_EN_PIN 12
#define MOTOR_LEFT_IN1_PIN 11
#define MOTOR_LEFT_IN2_PIN 10
#define MOTOR_RIGHT_EN_PIN 6
#define MOTOR_RIGHT_IN1_PIN 5
#define MOTOR_RIGHT_IN2_PIN 4

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

#define PWM_BITS 8  // PWM Resolution of the microcontroller


#define UPDATE_RATE_CONTROL 20
#define UPDATE_RATE_IMU 1
#define UPDATE_RATE_DEBUG 5

#define E_STOP_COMMAND_RECEIVED_DURATION 3 // Stop motors if no command was received after this amount of seconds

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(PWM_MAX)