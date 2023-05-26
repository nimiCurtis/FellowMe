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
#include <Encoder.h>
#include <L298N.h>


////motor control
//#include <Servo.h>
//Servo myservo;  // create servo object to control a servo
 
// Handles startup and shutdown of ROS
//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;

////////////////// Encoder & Velocities variables and consts /////////////////
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

long enc_left_prev_pos  = -999;
long enc_left_cur_pos;
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_LEFT_A 21 // white yellow 
// Other encoder output to Arduino to keep track of wheel direction
#define ENC_LEFT_B 20 //  yellow white
Encoder  Enc_left(ENC_LEFT_A, ENC_LEFT_B);



// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_RIGHT_A 18 // yellow
// Tracks the direction of rotation.
#define ENC_RIGHT_B 17  // white
Encoder  Enc_right(ENC_RIGHT_A, ENC_RIGHT_B); 
long enc_right_prev_pos  = -999;
long enc_right_cur_pos;


//geometric params
const int N = 140; // <<<check
const float R = 0.1016/2;
const float L = 0.296;
const float distance_per_count = (2*PI*R)/N;


// velocities
float vr_prev_raw = 0;
float vr_curr_filter = 0;

float vl_prev_raw = 0;
float vl_curr_filter = 0;

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
ros::Publisher enc_left_pub("/encoder/left/ticks", &enc_left_msg);

// right:
// encoder:
std_msgs::Int16 enc_right_msg;
ros::Publisher enc_right_pub("/encoder/right/ticks", &enc_right_msg);

 

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
int left_req_pwm = 0;
int right_req_pwm = 0;

// Motor Left connections
const unsigned int EN_left = 12; // white
const unsigned int IN1_left = 10; //yellow
const unsigned int IN2_left = 11; //brown
// Create left motor instance
L298N motor_left(EN_left, IN1_left, IN2_left);


// Motor Right connections
const unsigned int EN_right = 6; //white
const unsigned int IN1_right = 5; //yellow
const unsigned int IN2_right = 4; //brown
// Create right motor instance
L298N motor_right(EN_right, IN2_right, IN1_right);
 
// Record the time that the last velocity command was received
double left_last_pwm = 0;
double right_last_pwm  = 0; 

/////////////////////// Pwm subscribing ////////////////////////////

// Set up ROS subscriber to the pwm command
ros::Subscriber<std_msgs::Int16> pwm_left_sub("/motors/left/pwm" , &set_pwm_left);
ros::Subscriber<std_msgs::Int16> pwm_right_sub("/motors/right/pwm" , &set_pwm_right);

void set_pwm_left(const std_msgs::Int16& left_pwm_msg){
  left_last_pwm = (millis()/1000);
  left_req_pwm = left_pwm_msg.data;
  if(left_req_pwm > 0 ){
    motor_left.setSpeed(left_req_pwm);
    motor_left.forward();
  }
  
  else if(left_req_pwm < 0) {
    motor_left.setSpeed(-left_req_pwm);
    motor_left.backward();
 }
 else{
    motor_left.stop();
 }
//  pwmLeftReq = map(pwmLeftReq,-100, 100, 0, 180);
//  myservo.write(pwmLeftReq);
}

void set_pwm_right(const std_msgs::Int16& right_pwm_msg){
  right_last_pwm = (millis()/1000);
  right_req_pwm = right_pwm_msg.data;
  if(right_req_pwm >= 0 ){
    motor_right.setSpeed(right_req_pwm);
    motor_right.forward();
  }else {
    motor_right.setSpeed(-right_req_pwm);
    motor_right.backward();
 }
//  pwmLeftReq = map(pwmLeftReq,-100, 100, 0, 180);
//  myservo.write(pwmLeftReq);
}


/////////////////////// setup ////////////////////////////

void setup() {

//  myservo.attach(7);  // attaches the servo on pin 9 to the servo object

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
  nh.subscribe(pwm_left_sub);
  nh.subscribe(pwm_right_sub);
  

}

void loop() {
//
//  val = map(30,-100, 100, 0, 180);     // scale it for use with the servo (value between 0 and 180)
//  myservo.write(val);                  // sets the servo position according to the scaled value

  readEncoder_left();
  readEncoder_right();
  
  //unsigned long currentMillis = millis();
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6; // [seconds]
  
  

  //if (currentMillis-previousMillis >= 10){
  
  if (deltaT >= 0.01){  
  // Compute velocity with method 1
//    float vr_count = (pos_right - posPrev_right)/deltaT; // [count/sec]
//    posPrev_right = pos_right; 
////    float vl_count = (pos_left - posPrev_left)/deltaT; // [count/sec]
////    posPrev_left = pos_left; 

    

  // Convert count/s to RPM
    //float vl_rpm = vl_count/N*60.0; // [count/sec]/([count/rev]*[min/sec])
    //float vr_rpm = vr_count/N*60.0; // [count/sec]/([count/rev]*[min/sec])

  // Convert RPM to Linear speed of the wheels
//    float vl_curr_raw = vl_count*distance_per_count  ; // [count/sec]*[meters/count]
//    float vr_curr_raw = vr_count*distance_per_count   ; // [count/sec]*[meters/count]
  
  // Low-pass filter (25 Hz cutoff) <<< check values
//    vl_curr_filter = 0.9*vl_curr_filter + 0.05*vl_curr_raw + 0.05*vl_prev_raw;
//    vl_prev_raw = vl_curr_raw;

//    vr_curr_filter = 0.9*vr_curr_filter + 0.05*vr_curr_raw + 0.05*vr_prev_raw;
//    vr_prev_raw = vr_curr_raw;
//    

    //////////// set ROS msgs //////////////

    // set encoder msgs [counts]
    enc_left_msg.data = enc_left_cur_pos ; 
    enc_right_msg.data = enc_right_cur_pos ; 
    
     // Stop the car if there are no pwm messages in the last 3 sec
    if((millis()/1000) - left_last_pwm > 3) {   ////// changed to 3 from 1 sec
      motor_left.stop();
    }
    if((millis()/1000) - right_last_pwm > 3) {   ////// changed to 3 from 1 sec
      motor_right.stop();
    }
    
    //////////  publishing to topics //////
    
    enc_left_pub.publish( &enc_left_msg ); 
    enc_right_pub.publish( &enc_right_msg );


    // update previous time
    //startTimer();
    //previousMillis = currentMillis;
    prevT = currT;
  } 
 nh.spinOnce(); 
}
