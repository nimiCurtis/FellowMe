/// Encoder pins
// Left
#define ENCODER_LEFT_A 21
#define ENCODER_LEFT_B 20
/// Encoder pins
// Right
#define ENCODER_RIGHT_A 18
#define ENCODER_RIGHT_B 17

// Encoder resolution used for initialization 
// will be read from parameter server
#define ENCODER_RESOLUTION 540

// Wheels radius
#define WHEELS_RADIUS 0.1016/2

/// Motor pins
// Left
#define MOTOR_LEFT_EN 12 //white
#define MOTOR_LEFT_IN1 10 //brown
#define MOTOR_LEFT_IN2 11 //yellow
/// Motor pins
// Right
#define MOTOR_RIGHT_EN 6 //white
#define MOTOR_RIGHT_IN1 5 //yellow
#define MOTOR_RIGHT_IN2 4 //brown

#define K_P 15 // P constant
#define K_I 70 // I constant
#define K_D 0.5 // D constant

#define PWM_BITS 8  // PWM Resolution of the microcontroller

#define UPDATE_RATE_CONTROL 20
#define UPDATE_RATE_IMU 1
#define UPDATE_RATE_DEBUG 10

#define E_STOP_COMMAND_RECEIVED_DURATION 3 // Stop motors if no command was received after this amount of seconds

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(PWM_MAX)