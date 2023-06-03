#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include "EncoderHandler.h"
#include "MotorController.h"
#include "Timer.h"
#include "Constants.h"

ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;
sensor_msgs::JointState msg_measured_joint_states_;
ros::Publisher pub_measured_joint_states_("measured_joint_states", &msg_measured_joint_states_);
char* names[] = {"wheel_left", "wheel_right"};

EncoderHandler enc_left(ENC_LEFT_A, ENC_LEFT_B);
EncoderHandler enc_right(ENC_RIGHT_A, ENC_RIGHT_B);

MotorController motor_left(EN_LEFT, IN2_LEFT, IN1_LEFT);
MotorController motor_right(EN_RIGHT, IN2_RIGHT, IN1_RIGHT);

Timer timer;

int left_req_pwm = 0;
int right_req_pwm = 0;
double left_last_pwm = 0;
double right_last_pwm = 0;

void set_pwm_left(const std_msgs::Int16& pwm_msg) {
  left_req_pwm = pwm_msg.data;
}

void set_pwm_right(const std_msgs::Int16& pwm_msg) {
  right_req_pwm = pwm_msg.data;
}

void setup() {
  nh.initNode();
  nh.advertise(pub_measured_joint_states_);

  nh.subscribe("cmd_pwm_left", &set_pwm_left);
  nh.subscribe("cmd_pwm_right", &set_pwm_right);

  timer.start();
}

void loop() {
  nh.spinOnce();

  long left_encoder_pos, right_encoder_pos;
  enc_left.readEncoder(left_encoder_pos);
  enc_right.readEncoder(right_encoder_pos);

  float left_distance = left_encoder_pos * DISTANCE_PER_COUNT;
  float right_distance = right_encoder_pos * DISTANCE_PER_COUNT;

  float elapsed_time = timer.elapsedSeconds();

  float left_speed = (left_distance - left_last_pwm) / elapsed_time;
  float right_speed = (right_distance - right_last_pwm) / elapsed_time;

  left_last_pwm = left_distance;
  right_last_pwm = right_distance;

  msg_measured_joint_states_.header.stamp = nh.now();
  msg_measured_joint_states_.name_length = 2;
  msg_measured_joint_states_.position_length = 2;
  msg_measured_joint_states_.velocity_length = 2;

  msg_measured_joint_states_.name = names;
  msg_measured_joint_states_.position = new double[2];
  msg_measured_joint_states_.velocity = new double[2];

  msg_measured_joint_states_.position[0] = left_distance;
  msg_measured_joint_states_.position[1] = right_distance;

  msg_measured_joint_states_.velocity[0] = left_speed;
  msg_measured_joint_states_.velocity[1] = right_speed;

  pub_measured_joint_states_.publish(&msg_measured_joint_states_);

  motor_left.setSpeed(left_req_pwm);
  motor_right.setSpeed(right_req_pwm);
}

