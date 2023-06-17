#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <Encoder.h>
#include <L298N.h>
#include <fellowme_msgs/WheelsCmdStamped.h>
#include <fellowme_msgs/EncodersStamped.h>
#include <base_config.h>
#include <EncoderHandler.h>

using namespace fellowme_msgs;

// Function prototypes
void set_pwm(L298N motor, int pwm_val);
int pid_compute(double e, double e_d, double e_i);
void cmd_wheels_callback(const fellowme_msgs::WheelsCmdStamped& cmd_msg);

ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;

// ROS Publishers
sensor_msgs::JointState msg_measured_joint_states_;
ros::Publisher pub_measured_joint_states_("fellowme_base/measured_joint_states", &msg_measured_joint_states_);
char const* names[] = {"wheel_left", "wheel_right"};

// Encoder handler instances
EncoderHandler encoder_left(nh, ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RESOLUTION);
EncoderHandler encoder_right(nh, ENCODER_RIGHT_A, ENCODER_RIGHT_B, ENCODER_RESOLUTION);

fellowme_msgs::EncodersStamped encoders_msg;
ros::Publisher pub_encoders_("fellowme_base/encoders/ticks", &encoders_msg);

// Motor Controller Variables and Constants
int left_pwm = 0;
int right_pwm = 0;

// Motor Left connections
L298N motor_left(MOTOR_LEFT_EN, MOTOR_LEFT_IN2, MOTOR_LEFT_IN1);

// Motor Right connections
L298N motor_right(MOTOR_RIGHT_EN, MOTOR_RIGHT_IN2, MOTOR_RIGHT_IN1);

// Velocities
double left_angular_velocity_filter = 0;
double left_angular_velocity_prev = 0;
double right_angular_velocity_filter = 0;
double right_angular_velocity_prev = 0;

// Time interval for measurements in milliseconds
long previousMillis = 0;
long currentMillis = 0;
long prevT = 0;

// PID Constants
double pre_left_error = 0;
double left_i_error = 0;

double pre_right_error = 0;
double right_i_error = 0;

// Wheel Command Velocities
double wheel_cmd_velocity_left = 0;
double wheel_cmd_velocity_right = 0;

/////////////////////// Function Definitions ////////////////////////////

// Set the PWM value of the motor
void set_pwm(L298N motor, int pwm_val)
{
  if (pwm_val > 0)
  {
    motor.setSpeed(pwm_val);
    motor.forward();
  }
  else if (pwm_val < 0)
  {
    motor.setSpeed(-pwm_val);
    motor.backward();
  }
  else
  {
    motor.stop();
  }
}

// Compute the PID output based on the errors and constants
int pid_compute(double e, double e_d, double e_i)
{
  double pwm_output = K_P * e + K_D * e_d + K_I * e_i;

  // Clamp the PWM output within the allowed range
  if (pwm_output > PWM_MAX)
  {
    pwm_output = PWM_MAX;
  }
  else if (pwm_output < PWM_MIN)
  {
    pwm_output = PWM_MIN;
  }

  return pwm_output;
}

ros::Subscriber<fellowme_msgs::WheelsCmdStamped> cmd_wheels_sub("/fellowme/wheel_cmd_velocities", &cmd_wheels_callback);

// Callback function for the wheel command velocity subscriber
void cmd_wheels_callback(const fellowme_msgs::WheelsCmdStamped& cmd_msg)
{
  wheel_cmd_velocity_left = cmd_msg.wheels_cmd.angular_velocities.joint[0];
  wheel_cmd_velocity_right = cmd_msg.wheels_cmd.angular_velocities.joint[1];
}

void setup()
{
  encoder_left.setup();
  encoder_right.setup();

  msg_measured_joint_states_.position = (float*)malloc(sizeof(float) * 2);
  msg_measured_joint_states_.position_length = 2;
  msg_measured_joint_states_.position[0] = 0;
  msg_measured_joint_states_.position[1] = 0;
  msg_measured_joint_states_.velocity = (float*)malloc(sizeof(float) * 2);
  msg_measured_joint_states_.velocity_length = 2;
  msg_measured_joint_states_.name_length = 2;
  msg_measured_joint_states_.effort = (float*)malloc(sizeof(float) * 2);
  msg_measured_joint_states_.effort_length = 2;

  nh.advertise(pub_measured_joint_states_);
  motor_left.stop();
  motor_right.stop();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_encoders_);
  nh.subscribe(cmd_wheels_sub);
}

void loop()
{
  int left_ticks = encoder_left.getTicks();
  int right_ticks = encoder_right.getTicks();

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;

  if (deltaT >= 0.05)
  {
    JointState joint_state_left = encoder_left.getJointState();
    JointState joint_state_right = encoder_right.getJointState();

    msg_measured_joint_states_.position[0] += joint_state_left.angular_position_;
    // if (msg_measured_joint_states_.position[0] > 2 * PI)
    // {
    //   msg_measured_joint_states_.position[0] -= 2 * PI;
    // }
    // if (msg_measured_joint_states_.position[0] < 0)
    // {
    //   msg_measured_joint_states_.position[0] += 2 * PI;
    // }

    left_angular_velocity_filter = 0.88 * left_angular_velocity_filter + 0.06 * joint_state_left.angular_velocity_ + 0.06 * left_angular_velocity_prev;
    left_angular_velocity_prev = joint_state_left.angular_velocity_;

    msg_measured_joint_states_.velocity[0] = left_angular_velocity_filter;

    msg_measured_joint_states_.position[1] += joint_state_right.angular_position_;
    // if (msg_measured_joint_states_.position[1] > 2 * PI)
    // {
    //   msg_measured_joint_states_.position[1] -= 2 * PI;
    // }
    // if (msg_measured_joint_states_.position[1] < 0)
    // {
    //   msg_measured_joint_states_.position[1] += 2 * PI;
    // }

    right_angular_velocity_filter = 0.88 * right_angular_velocity_filter + 0.06 * joint_state_right.angular_velocity_ + 0.06 * right_angular_velocity_prev;
    right_angular_velocity_prev = joint_state_right.angular_velocity_;

    msg_measured_joint_states_.velocity[1] = right_angular_velocity_filter;

    // Control the left wheel
    double left_error = wheel_cmd_velocity_left - left_angular_velocity_filter;
    double left_d_error = (left_error - pre_left_error) / deltaT;
    left_i_error += left_error * deltaT;
    pre_left_error = left_error;

    if (wheel_cmd_velocity_left == 0)
    {
      set_pwm(motor_left, 0);
    }
    else
    {
      double left_pwm = pid_compute(left_error, left_d_error, left_i_error);
      set_pwm(motor_left, (int)left_pwm);
    }

    // Control the right wheel
    double right_error = wheel_cmd_velocity_right - right_angular_velocity_filter;
    double right_d_error = (right_error - pre_right_error) / deltaT;
    right_i_error += right_error * deltaT;
    pre_right_error = right_error;

    if (wheel_cmd_velocity_right == 0)
    {
      set_pwm(motor_right, 0);
    }
    else
    {
      double right_pwm = pid_compute(right_error, right_d_error, right_i_error);
      set_pwm(motor_right, (int)right_pwm);
    }

    msg_measured_joint_states_.name = names;
    msg_measured_joint_states_.effort[0] = left_pwm;
    msg_measured_joint_states_.effort[1] = right_pwm;

    encoders_msg.encoders.ticks[0] = left_ticks;
    encoders_msg.encoders.ticks[1] = right_ticks;

    pub_encoders_.publish(&encoders_msg);
    pub_measured_joint_states_.publish(&msg_measured_joint_states_);

    prevT = currT;
  }
  nh.spinOnce();
}




