#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <Encoder.h>
#include <fellowme_msgs/WheelsCmdStamped.h>
#include <fellowme_msgs/EncodersStamped.h>
#include <L298NController.h>
#include <base_config.h>
#include <EncoderHandler.h>
#include <pid.h>
// #include <base_controller.h>
using namespace fellowme_msgs;

// Function prototypes
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

// Motor Left connections
L298NController motor_left(MOTOR_LEFT_EN, MOTOR_LEFT_IN2, MOTOR_LEFT_IN1);
fellowme::PID motor_pid_left_(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Motor Right connections 
L298NController motor_right(MOTOR_RIGHT_EN, MOTOR_RIGHT_IN2, MOTOR_RIGHT_IN1);
fellowme::PID motor_pid_right_(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Motor Controller Variables and Constants
int left_pwm = 0;
int right_pwm = 0;

// Velocities
double left_angular_velocity_filter = 0;
double left_angular_velocity_prev = 0;
double right_angular_velocity_filter = 0;
double right_angular_velocity_prev = 0;



// Wheel Command Velocities
double wheel_cmd_velocity_left = 0;
double wheel_cmd_velocity_right = 0;

ros::Subscriber<fellowme_msgs::WheelsCmdStamped> cmd_wheels_sub("/fellowme/wheel_cmd_velocities", &cmd_wheels_callback);
// Time interval for measurements in milliseconds
ros::Time prevT = nh.now();
// float prevT = 0;
ros::Time command_t;

// Callback function for the wheel command velocity subscriber
void cmd_wheels_callback(const fellowme_msgs::WheelsCmdStamped& cmd_msg)
{
  wheel_cmd_velocity_left = cmd_msg.wheels_cmd.angular_velocities.joint[0];
  wheel_cmd_velocity_right = cmd_msg.wheels_cmd.angular_velocities.joint[1];
  command_t = nh.now();
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
  motor_left.setSpeed(0);
  motor_right.setSpeed(0);



  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_encoders_);
  nh.subscribe(cmd_wheels_sub);
}

void loop()
{
  int left_ticks = encoder_left.getTicks();
  int right_ticks = encoder_right.getTicks();

  ros::Time currT = nh.now();
  ros::Duration dt = currT - prevT;
  ros::Duration command_dt = currT - command_t;
  if (command_dt.toSec() > E_STOP_COMMAND_RECEIVED_DURATION){
    wheel_cmd_velocity_left = 0;
    wheel_cmd_velocity_right = 0;
  }
  // float deltaT = ((currT - prevT)) / 1.0e6;

  // if (deltaT >= 0.05)
  if (dt.toSec() >= 0.05)
  {
    JointState joint_state_left = encoder_left.getJointState();
    JointState joint_state_right = encoder_right.getJointState();

    msg_measured_joint_states_.position[0] += joint_state_left.angular_position_;

    left_angular_velocity_filter = 0.88 * left_angular_velocity_filter + 0.06 * joint_state_left.angular_velocity_ + 0.06 * left_angular_velocity_prev;
    left_angular_velocity_prev = joint_state_left.angular_velocity_;

    msg_measured_joint_states_.velocity[0] = left_angular_velocity_filter;

    msg_measured_joint_states_.position[1] += joint_state_right.angular_position_;

    right_angular_velocity_filter = 0.88 * right_angular_velocity_filter + 0.06 * joint_state_right.angular_velocity_ + 0.06 * right_angular_velocity_prev;
    right_angular_velocity_prev = joint_state_right.angular_velocity_;

    msg_measured_joint_states_.velocity[1] = right_angular_velocity_filter;


    // Control the left wheel
    double left_pwm = motor_pid_left_.compute(wheel_cmd_velocity_left, left_angular_velocity_filter,dt.toSec());
    motor_left.setSpeed((int)left_pwm);
    
    
    // Control the right wheel
    double right_pwm = motor_pid_right_.compute(wheel_cmd_velocity_right, right_angular_velocity_filter,dt.toSec());
    motor_right.setSpeed((int)right_pwm);

    // set effort and names
    msg_measured_joint_states_.name = names;
    msg_measured_joint_states_.effort[0] = left_pwm;
    msg_measured_joint_states_.effort[1] = right_pwm;

    // set encoders msg
    encoders_msg.encoders.ticks[0] = left_ticks;
    encoders_msg.encoders.ticks[1] = right_ticks;

    // publish msgs
    pub_encoders_.publish(&encoders_msg);
    pub_measured_joint_states_.publish(&msg_measured_joint_states_);

    prevT = currT;
  }
  nh.spinOnce();
}


