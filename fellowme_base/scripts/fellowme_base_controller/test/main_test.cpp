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
#include <base_controller.h>

ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;
// ros::NodeHandle 

using namespace fellowme;

L298NController motor_controller_left(MOTOR_LEFT_EN, MOTOR_LEFT_IN2, MOTOR_LEFT_IN1);
L298NController motor_controller_right(MOTOR_RIGHT_EN, MOTOR_RIGHT_IN2, MOTOR_RIGHT_IN1);


BaseController base_controller(nh, &motor_controller_left, &motor_controller_right);


void setup()
{
    base_controller.setup();
    base_controller.init();

    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.setSpeed(0);
    motor_controller_right.setSpeed(0);
    nh.loginfo("Setup finished");
}


void loop()
{
    static bool imu_is_initialized;

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

    // This block publishes the IMU data based on a defined imu rate
    ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
    // if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
    // {
    //     // Sanity check if the IMU is connected
    //     if (!imu_is_initialized)
    //     {
    //         //imu_is_initialized = initIMU();
    //         if(imu_is_initialized)
    //             nh.loginfo("IMU Initialized");
    //         else
    //             nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    //     }
    //     else
    //     {
    //         //publishIMU();
    //     }
    //     base_controller.lastUpdateTime().imu = nh.now();
    // }

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




