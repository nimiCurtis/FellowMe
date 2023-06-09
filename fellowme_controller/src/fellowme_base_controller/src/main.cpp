/*
 * Reference:
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly. 
 */
 
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <Encoder.h>
#include <L298N.h>
#include <fellowme_msgs/WheelsCmdStamped.h>
#include <base_config.h>

using namespace fellowme_msgs;

void set_pwm_left_msg(const std_msgs::Int16& left_pwm_msg);
void set_pwm_right_msg(const std_msgs::Int16& right_pwm_msg);
void readEncoder_left();
void readEncoder_right();
void set_pwm(L298N motor, int pwm_val);
int pid_compute(double e, double e_d,double e_i);
void cmd_wheels_callback(const fellowme_msgs::WheelsCmdStamped& cmd_msg);

// Handles startup and shutdown of ROS
//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;


sensor_msgs::JointState msg_measured_joint_states_;
ros::Publisher pub_measured_joint_states_("fellowme/fellowme_base/joint_states", &msg_measured_joint_states_);
char const *names[] = {"wheel_left", "wheel_right"};


////////////////// Encoder & Velocities variables and consts /////////////////
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

long enc_left_prev_pos  = -999;
long enc_left_cur_pos;
long penc_left = 0;
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
Encoder  Enc_left(ENCODER_LEFT_A, ENCODER_LEFT_B);



// Encoder output to Arduino Interrupt pin. Tracks the tick count.
Encoder  Enc_right(ENCODER_RIGHT_A, ENCODER_RIGHT_B); 
long enc_right_prev_pos  = -999;
long enc_right_cur_pos;
long penc_right = 0;


// velocities
double left_angular_velocity_filter = 0;
double left_angular_velocity_prev = 0;
double right_angular_velocity_filter = 0;
double right_angular_velocity_prev = 0;

// Time interval for measurements in milliseconds
long previousMillis = 0;
long currentMillis = 0;
// previous time in micros << check if necessary
long prevT = 0;

int val;    // variable to read the value from the analog pin

////////// Encoders & velocities & distance ROS topics //////

// left:
// encoder:
std_msgs::Int16 enc_left_msg;
ros::Publisher enc_left_pub("fellowme/fellowme_base/encoder/left/ticks", &enc_left_msg);

// right:
// encoder:
std_msgs::Int16 enc_right_msg;
ros::Publisher enc_right_pub("fellowme/fellowme_base/encoder/right/ticks", &enc_right_msg);

 

/////////////////////// encoders handle functions ////////////////
// left:
void readEncoder_left(){
  enc_left_cur_pos = Enc_left.read();
  if (enc_left_cur_pos != enc_left_prev_pos) {
    enc_left_prev_pos = enc_left_cur_pos;
  }
  
}

// right:
void readEncoder_right(){
  enc_right_cur_pos = Enc_right.read();
  
  if (enc_right_cur_pos != enc_right_prev_pos) {
    enc_right_prev_pos = enc_right_cur_pos;
  }
}


////////////////// Motor Controller Variables and Constants ///////////////////
int left_pwm = 0;
int right_pwm = 0;

// Motor Left connections
// Create left motor instance
L298N motor_left(MOTOR_LEFT_EN, MOTOR_LEFT_IN2, MOTOR_LEFT_IN1);


// Motor Right connections
// Create right motor instance
L298N motor_right(MOTOR_RIGHT_EN, MOTOR_RIGHT_IN2, MOTOR_RIGHT_IN1);
 
// Record the time that the last velocity command was received
double left_last_pwm = 0;
double right_last_pwm  = 0; 

void set_pwm(L298N motor, int pwm_val){
  if(pwm_val > 0 ){
    motor.setSpeed(pwm_val);
    motor.forward();
  }
  
  else if(pwm_val < 0) {
    motor.setSpeed(-pwm_val);
    motor.backward();
 }
  else{
    motor.stop();
 }
}

double wheel_cmd_velocity_left = 0;
double wheel_cmd_velocity_right = 0;

double pre_left_error = 0;
double left_i_error = 0;

double pre_right_error = 0;
double right_i_error = 0;


// //write the pid function
int pid_compute(double e, double e_d,double e_i){
  double pwm_output = K_P*e + K_D*e_d + K_I*e_i; //use the pid equation and the constants
  // clamp the pwm if it is above the MAX_PWM or below the MIN_PWM 
  if (pwm_output > PWM_MAX){
    pwm_output = PWM_MAX;
  }
  else if(pwm_output<PWM_MIN){
    pwm_output = PWM_MIN;
  }
  return pwm_output;
}

/////////////////////// wheel_cmd subscribing ////////////////////////////
// Set up ROS subscriber to the wheel velocity command

ros::Subscriber<fellowme_msgs::WheelsCmdStamped> cmd_wheels_sub("fellowme/fellowme_base/motors/cmd_wheels", &cmd_wheels_callback);

void cmd_wheels_callback(const fellowme_msgs::WheelsCmdStamped& cmd_msg){
  // Callback function every time the angular wheel commands for each wheel joint are received from 'wheel_cmd_velocities' topic
    // This callback function receives fellowme_msgs::WheelsCmdStamped message object
    // where fellowme_msgs::AngularVelocities for both joints are stored
    wheel_cmd_velocity_left = cmd_msg.wheels_cmd.angular_velocities.joint[0];
    wheel_cmd_velocity_right = cmd_msg.wheels_cmd.angular_velocities.joint[1];

}


/////////////////////// setup ////////////////////////////

void setup() {

  msg_measured_joint_states_.position = (float*)malloc(sizeof(float) * 2);
  msg_measured_joint_states_.position_length = 2;
  msg_measured_joint_states_.position[0] = 0;
  msg_measured_joint_states_.position[1] = 0;
  msg_measured_joint_states_.velocity = (float*)malloc(sizeof(float) * 2);
  msg_measured_joint_states_.velocity_length = 2;
//  msg_measured_joint_states_.name = (char*)malloc(sizeof(char) * 2);
  msg_measured_joint_states_.name_length = 2;
  msg_measured_joint_states_.effort = (float*)malloc(sizeof(float) * 2);
  msg_measured_joint_states_.effort_length = 2;

  nh.advertise(pub_measured_joint_states_);
  // Set the motors intial speed - starts from 'stop' state
  motor_left.stop();
  motor_right.stop();
  
  
  // ROS Setup
  nh.getHardware()->setBaud(115200); ///changed from 57600, 115200 , 9200, 74880, 38400, 19200, 4800
  nh.initNode();

  //publisher:
  nh.advertise(enc_left_pub);
  nh.advertise(enc_right_pub);

  //subscribers:
  nh.subscribe(cmd_wheels_sub);

}

void loop() {

  readEncoder_left();
  readEncoder_right();
  
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6; // [seconds]
  
  if (deltaT >= 0.05){
    
    //left
    long delta_ticks_left = enc_left_cur_pos - penc_left;

    double delta_angle_left = (double)delta_ticks_left * ((2*PI)/ENCODER_RESOLUTION);
    double left_angular_velocity = delta_angle_left/deltaT;
    
    msg_measured_joint_states_.position[0] += delta_angle_left;
    if (msg_measured_joint_states_.position[0] > 2*PI){
      (msg_measured_joint_states_.position[0] -= 2*PI);
    }
    if (msg_measured_joint_states_.position[0] < 0){
      msg_measured_joint_states_.position[0] += 2*PI;
    }

    left_angular_velocity_filter = 0.92*left_angular_velocity_filter + 0.04*left_angular_velocity + 0.04*left_angular_velocity_prev;
    left_angular_velocity_prev = left_angular_velocity;
    
    msg_measured_joint_states_.velocity[0] = left_angular_velocity_filter;


    //right
    long delta_ticks_right = enc_right_cur_pos - penc_right;

    double delta_angle_right = (double)delta_ticks_right * ((2*PI)/ENCODER_RESOLUTION);
    double right_angular_velocity = delta_angle_right/deltaT;
    
    msg_measured_joint_states_.position[1] += delta_angle_right;
    if (msg_measured_joint_states_.position[1] > 2*PI){
      msg_measured_joint_states_.position[1] -= 2*PI;
    }
    if (msg_measured_joint_states_.position[1] < 0){
      msg_measured_joint_states_.position[1] += 2*PI;
    }

    right_angular_velocity_filter = 0.92*right_angular_velocity_filter + 0.04*right_angular_velocity + 0.04*right_angular_velocity_prev;
    right_angular_velocity_prev = right_angular_velocity;
    
    msg_measured_joint_states_.velocity[1] = right_angular_velocity_filter;


    //control left
    
    double left_error = wheel_cmd_velocity_left - left_angular_velocity_filter;  //calc velocity error between target angular velocity and current velocity 
    double left_d_error = (left_error - pre_left_error)/deltaT;//calc the derivative velocity -> hint: deltaT = time difference
    left_i_error +=  left_error*deltaT; //calc the integral derivative -> hint: deltaT = time difference
    pre_left_error = left_error;
    
    if (wheel_cmd_velocity_left==0){
      set_pwm(motor_left,0);
    }
    else{
      double left_pwm = pid_compute(left_error,left_d_error, left_i_error);// compute the controled pwm value -> hint: use the pid_compute function
      set_pwm(motor_left,(int)left_pwm);
    }
      

    
    //control right
    
    double right_error = wheel_cmd_velocity_right - right_angular_velocity_filter;  //calc velocity error between target angular velocity and current velocity 
    double right_d_error = (right_error - pre_right_error)/deltaT;//calc the derivative velocity -> hint: deltaT = time difference
    right_i_error +=  right_error*deltaT; //calc the integral derivative -> hint: deltaT = time difference
    pre_right_error = right_error;
    
    if (wheel_cmd_velocity_right==0){
      set_pwm(motor_right,0);
    }
    else{
      double right_pwm = pid_compute(right_error,right_d_error, right_i_error);// compute the controled pwm value -> hint: use the pid_compute function
      set_pwm(motor_right,(int)right_pwm);
    }
    
    msg_measured_joint_states_.name = names;
    msg_measured_joint_states_.effort[0] = left_pwm;
    msg_measured_joint_states_.effort[1] = right_pwm;


    //////////// set ROS msgs //////////////

    // set encoder msgs [counts]
    enc_left_msg.data = enc_left_cur_pos ; 
    enc_right_msg.data = enc_right_cur_pos ; 
    
    //////////  publishing to topics //////
    
    enc_left_pub.publish( &enc_left_msg ); 
    enc_right_pub.publish( &enc_right_msg );
    pub_measured_joint_states_.publish( &msg_measured_joint_states_);

    penc_left = enc_left_cur_pos ; 
    penc_right = enc_right_cur_pos ; 

    // update previous time
    //startTimer();
    //previousMillis = currentMillis;
    prevT = currT;
  } 
 nh.spinOnce(); 
}
