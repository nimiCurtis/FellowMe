#include "EncoderHandler.h"
#include <ros.h>

EncoderHandler::EncoderHandler(ros::NodeHandle& nh, int pinA, int pinB, int ticksPerRevolution)
    : nh_(nh),
      encoder_(pinA, pinB),
      ticksPerRevolution_(ticksPerRevolution),
      ticks_(0),
      ticksPrev_(0),
      prevUpdateTime_(0, 0)
{
}

void EncoderHandler::setup()
{
    // Reset the encoder count to zero
    encoder_.write(0);
}

int EncoderHandler::getTicks()
{
    // Read the current encoder count
    int ticks = encoder_.read();

    return ticks;
}

int EncoderHandler::getDeltaTicks()
{
    // Calculate the change in encoder ticks since the last update
    int deltaTicks = ticks_ - ticksPrev_;

    return deltaTicks;
}

JointState EncoderHandler::getJointState()
{
    // Read the current encoder ticks
    long encoderTicks = encoder_.read();

    // Calculate the delta time
    ros::Time currentTime = nh_.now();
    ros::Duration deltaTime = currentTime - prevUpdateTime_;

    // Convert the delta time to seconds
    double deltaTimeSec = deltaTime.toSec();

    // Calculate delta angle and angular velocity
    double deltaTicks = encoderTicks - ticksPrev_;
    double deltaAngle = computeDeltaAngle(deltaTicks);
    double angularVelocity = deltaAngle / deltaTimeSec;

    // Update the joint state
    joint_state_.angular_position_ = deltaAngle;
    joint_state_.angular_velocity_ = angularVelocity;

    // Update previous values
    prevUpdateTime_ = currentTime;
    ticksPrev_ = encoderTicks;

    return joint_state_;
}

double EncoderHandler::computeDeltaAngle(int deltaTicks)
{
    // Calculate the change in angle based on the encoder ticks and ticks per revolution
    double deltaAngle = static_cast<double>(deltaTicks) * ((2 * PI) / ticksPerRevolution_);

    return deltaAngle;
}

void EncoderHandler::write(int32_t p)
{
    // set the encoder count to value of p
    encoder_.write(p);
}
