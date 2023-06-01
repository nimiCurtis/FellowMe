//#include <ros.h>
#include "ros.h"
#include "fellowme_base_config.h"
#include "base_controller.h"
#include "dfr0601/dfr0601.h"

ros::NodeHandle nh;

using namespace fellowme;

Dfr0601MotorController motor_controller_left = Dfr0601MotorController(MOTOR_LEFT_EN_PIN,MOTOR_LEFT_IN1_PIN,MOTOR_LEFT_IN2_PIN);
Dfr0601MotorController motor_controller_right = Dfr0601MotorController(MOTOR_RIGHT_EN_PIN,MOTOR_RIGHT_IN1_PIN,MOTOR_RIGHT_IN2_PIN);

//wtf???
BaseController<Dfr0601MotorController, L298N> base_controller(nh, &motor_controller_left, &motor_controller_right);


void setup()
{
    base_controller.setup();
    base_controller.init();

    nh.loginfo("Initialize fellowme Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
}


void loop()
{

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
    if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = nh.now();
    }

    // This block stops the motor when no wheel command is received
    // from the high level hardware_interface::RobotHW
    command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
    if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
    {
        nh.logwarn("Emergency STOP");
        base_controller.eStop();
    }


    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
        if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = nh.now();
        }
    }
    // Call all the callbacks waiting to be called
    nh.spinOnce();
}