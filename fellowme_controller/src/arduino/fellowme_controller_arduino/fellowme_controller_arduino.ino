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

////motor control
//#include <Servo.h>
//Servo myservo;  // create servo object to control a servo


//velocity calculation
#include <util/atomic.h>
 
// Handles startup and shutdown of ROS
//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;



////////////////// Encoder & Velocities variables and consts /////////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 20 //white 
#define ENC_IN_RIGHT_A 18 //
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 21 // blue 
#define ENC_IN_RIGHT_B 19  // 
 
//geometric params
const int N = 140; // <<<check
const float R = 0.1016/2;
const float L = 0.296;
const float distance_per_count = (2*PI*R)/N;


// Encoders 
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i_left = 0;
volatile long prevT_i_left = 0;
int posPrev_left = 0;

volatile int pos_i_right = 0;
volatile long prevT_i_right = 0;
int posPrev_right = 0;

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

// right:
// encoder:
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("/encoder_right_ticks", &right_wheel_tick_count);

// left:
// encoder:
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("/encoder_left_ticks", &left_wheel_tick_count);
 

/////////////////////// encoders handle functions ////////////////
// left:
void readEncoder_left(){
  // Read encoder B when ENCA rises
  int b_left = digitalRead(ENC_IN_LEFT_B);
  int increment_left = 0;
  if(b_left>0){
    // If B is high, increment backward
    increment_left = 1;
  }
  else{
    // Otherwise, increment forward
    increment_left = -1;
  }
  pos_i_left = pos_i_left + increment_left;
}

// right:
void readEncoder_right(){
  // Read encoder B when ENCA rises
  int b_right = digitalRead(ENC_IN_RIGHT_B);
  int increment_right = 0;
  if(b_right>0){
    // If B is high, increment forward
    increment_right = 1;
  }
  else{
    // Otherwise, increment backward 
    increment_right = -1;
  }
  pos_i_right = pos_i_right + increment_right;
}




////////////////// Motor Controller Variables and Constants ///////////////////
int pwmLeftReq = 0;
int pwmRightReq = 0;

// Motor A connections - left
const int pwmA = 12 ; //white
const int in1 = 11 ; // orange
const int in2 = 10 ; // purple
  
// Motor B connections - right
const int pwmB = 6; //white 
const int in3 = 5; //orange
const int in4 = 4; //purple
 
// Record the time that the last velocity command was received
double left_lastPwmReceived = 0;
double right_lastPwmReceived = 0; 

/////////////////////// Pwm subscribing ////////////////////////////

void set_pwm_right(const std_msgs::Int16& right_pwm_out){
  right_lastPwmReceived = (millis()/1000);
  pwmRightReq = right_pwm_out.data;
  set_pwm_values(pwmRightReq, pwmB , in3 , in4);
//  pwmRightReq = map(pwmRightReq,-100, 100, 0, 180);
//  myservo.write(pwmRightReq);
}
 
void set_pwm_left(const std_msgs::Int16& left_pwm_out){
  left_lastPwmReceived = (millis()/1000);
  pwmLeftReq = left_pwm_out.data;
  set_pwm_values(pwmLeftReq, pwmA , in1 , in2);
//  pwmLeftReq = map(pwmLeftReq,-100, 100, 0, 180);
//  myservo.write(pwmLeftReq);
}
 
void set_pwm_values(int pwm_val ,int pwm_pin ,int in1_pin ,int in2_pin) {
 if(pwm_val >= 0 ){
    digitalWrite(in1_pin, LOW); // setting direction
    digitalWrite(in2_pin, HIGH);
    analogWrite(pwm_pin, pwm_val);
 }else {
    pwm_val = -1*pwm_val;
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW );
    analogWrite(pwm_pin, pwm_val);
 }
}

// Set up ROS subscriber to the pwm command
ros::Subscriber<std_msgs::Int16> subLeftPwm("/left_motor_pwm" , &set_pwm_left);
ros::Subscriber<std_msgs::Int16> subRightPwm("/right_motor_pwm" , &set_pwm_right);


/////////////////////// setup ////////////////////////////

void setup() {

//  myservo.attach(7);  // attaches the servo on pin 9 to the servo object


  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT); 
  
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A),readEncoder_left,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A),readEncoder_right,RISING);

  // Motor control pins setup
  // left motor pin setup
  pinMode(pwmA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // right motor pin setup
  pinMode(pwmB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set the motor speed
  analogWrite(pwmA, 0); 
  analogWrite(pwmB, 0);
 
  // ROS Setup
  nh.getHardware()->setBaud(115200); ///changed from 57600, 115200 , 9200, 74880, 38400, 19200, 4800
  nh.initNode();

  nh.advertise(rightPub);

  nh.advertise(leftPub);

  nh.subscribe(subLeftPwm);
  nh.subscribe(subRightPwm);
  

}

void loop() {
//
//  val = map(30,-100, 100, 0, 180);     // scale it for use with the servo (value between 0 and 180)
//  myservo.write(val);                  // sets the servo position according to the scaled value


  //unsigned long currentMillis = millis();
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6; // [seconds]
  
  
  //if (currentMillis-previousMillis >= 10){
  if (deltaT >= 0.01){  
    //Serial.println(deltaT);
  // read the position in an atomic block
  // to avoid potential misreads
    int pos_right = 0;
    int pos_left = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      pos_left = pos_i_left;
      pos_right = pos_i_right;
    }
    
  // Compute velocity with method 1
    
    
    
    
    float vr_count = (pos_right - posPrev_right)/deltaT; // [count/sec]
    posPrev_right = pos_right; 
    float vl_count = (pos_left - posPrev_left)/deltaT; // [count/sec]
    posPrev_left = pos_left; 

    

  // Convert count/s to RPM
    //float vl_rpm = vl_count/N*60.0; // [count/sec]/([count/rev]*[min/sec])
    //float vr_rpm = vr_count/N*60.0; // [count/sec]/([count/rev]*[min/sec])

  // Convert RPM to Linear speed of the wheels
    float vl_curr_raw = vl_count*distance_per_count  ; // [count/sec]*[meters/count]
    float vr_curr_raw = vr_count*distance_per_count   ; // [count/sec]*[meters/count]
  
  // Low-pass filter (25 Hz cutoff) <<< check values
    vl_curr_filter = 0.9*vl_curr_filter + 0.05*vl_curr_raw + 0.05*vl_prev_raw;
    vl_prev_raw = vl_curr_raw;

    vr_curr_filter = 0.9*vr_curr_filter + 0.05*vr_curr_raw + 0.05*vr_prev_raw;
    vr_prev_raw = vr_curr_raw;
    
  
  // calc distances
    float delta_right_distance = vr_curr_filter*deltaT ; // right wheel distance passed [meter/sec]*[sec] 
    float delta_left_distance = vl_curr_filter*deltaT ; // left wheel distance passed [meter/sec]*[sec]
    //float delta_distance = (delta_right_distance + delta_left_distance)/2 ; // total distance robot passed [meter]
    //float delta_theta = (delta_right_distance - delta_left_distance)/L;
    

    

    
    //////////// set ROS msgs //////////////

    // set encoder msgs [counts]
    left_wheel_tick_count.data = pos_left ; 
    right_wheel_tick_count.data = pos_right ; 
    

      
     // Stop the car if there are no pwm messages in the last 3 sec
    if((millis()/1000) - left_lastPwmReceived > 3) {   ////// changed to 3 from 1 sec
      pwmLeftReq = 0;
      analogWrite(pwmA, 0);
    }
    if((millis()/1000) - right_lastPwmReceived > 3) {   ////// changed to 3 from 1 sec
      pwmRightReq = 0;
      analogWrite(pwmB, 0);
    }
    
    //////////  publishing to topics //////
    
    leftPub.publish( &left_wheel_tick_count ); 
    rightPub.publish( &right_wheel_tick_count );
    

    
    
    // update previous time
    //startTimer();
    //previousMillis = currentMillis;
    prevT = currT;
  } 
 nh.spinOnce(); 
}
