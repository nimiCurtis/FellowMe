#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Arduino.h>
#include <Encoder.h>
#include <ros.h>

struct JointState {
  double angular_position_;
  double angular_velocity_;
};

class EncoderHandler {
public:
  /**
   * @brief Constructor for EncoderHandler class.
   * @param nh The ROS node handle.
   * @param pinA The pin connected to channel A of the encoder.
   * @param pinB The pin connected to channel B of the encoder.
   * @param ticksPerRevolution The number of ticks per revolution of the encoder.
   */
  EncoderHandler(ros::NodeHandle& nh, int pinA, int pinB, int ticksPerRevolution);

  /**
   * @brief Setup function to initialize the encoder.
   */
  void setup();

  /**
   * @brief Get the current tick count from the encoder.
   * @return The current tick count.
   */
  int getTicks();

  /**
   * @brief Get the change in tick count since the last update.
   * @return The change in tick count.
   */
  int getDeltaTicks();

  /**
   * @brief Calculate the delta angle based on the change in tick count.
   * @param deltaTicks The change in tick count.
   * @return The delta angle in radians.
   */
  double computeDeltaAngle(int deltaTicks);

  /**
   * @brief Get the joint state including angular position and velocity.
   * @return The joint state.
   */
  JointState getJointState();

private:
  ros::NodeHandle& nh_;       // ROS node handle
  Encoder encoder_;           // Encoder object
  int ticksPerRevolution_;    // Ticks per revolution of the encoder
  volatile int ticks_;        // Current tick count
  volatile int ticksPrev_;    // Previous tick count
  JointState joint_state_;    // Current joint state (position and velocity)
  ros::Time prevUpdateTime_;  // Previous update time for calculating angular velocity
};

#endif
