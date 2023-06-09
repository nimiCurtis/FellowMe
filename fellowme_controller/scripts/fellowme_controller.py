#!/usr/bin/env python3

# import libraries & ros plugins & msgs
import numpy as np
import rospy
from math import pi, cos, sin, asin
from std_msgs.msg import Float64, Float32, Int16
from fellowme_msgs.srv import PwmVal, PwmValResponse, WheelsCmdSrv, WheelsCmdSrvResponse, WheelsCmdSrvRequest
from fellowme_msgs.msg import WheelsCmdStamped, WheelsCmd, AngularVelocities
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from tf.broadcaster import TransformBroadcaster
import time


class Controller:
    """Controller class for the FellowMe robot."""

    def __init__(self):
        """Initialize the Controller class."""
        
        # Publisher for commanding wheel speeds
        self.cmd_wheels_pub = rospy.Publisher("fellowme/fellowme_base/motors/cmd_wheels", WheelsCmdStamped, queue_size=100)
        self.cmd_wheels_msg = WheelsCmdStamped()
        self.cmd_wheels_msg.wheels_cmd = WheelsCmd(AngularVelocities([0, 0]))
        
        # Subscriber for cmd_vel messages
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # Publisher for odometry messages
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=1000)
        
        # Initialize time variables
        self.t_0 = time.time()
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        # Initialize PWM parameters for left and right wheels
        self.pwm_left_out = Int16()
        self.pwm_left_out.data = 0
        
        self.pwm_right_out = Int16()
        self.pwm_right_out.data = 0
        
        # Initialize kinematics parameters
        self.R = 0.1016/2
        self.L = 0.296
        
        # Initialize odometry and tf
        self.odom = Odometry()
        self.odom_broadcaster = TransformBroadcaster()
        
        # Initialize shutdown hook process
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
    
    def shutdownhook(self):
        """Shutdown hook process to be executed on node shutdown."""
        rospy.loginfo("shutting down")
        self.ctrl_c = True
        self.stop()
    
    def publish(self):
        """Publish the commanded wheel speeds."""
        self.cmd_wheels_msg.header.stamp = rospy.Time.now()
        self.cmd_wheels_msg.header.frame_id = ''
        self.cmd_wheels_pub.publish(self.cmd_wheels_msg)
    
    def cmd_vel_callback(self, msg):
        """Callback function for the cmd_vel subscriber."""
        cmd_vel = msg
        cmd_right = (2 * cmd_vel.linear.x + cmd_vel.angular.z * self.L) / (2 * self.R)
        cmd_left = (2 * cmd_vel.linear.x - cmd_vel.angular.z * self.L) / (2 * self.R)
        self.cmd_wheels_msg.wheels_cmd = WheelsCmd(AngularVelocities([cmd_left, cmd_right]))
    
    def cmd_wheels_srv_callback(self, request: WheelsCmdSrvRequest):
        """Callback function for the cmd_wheels service."""
        response = WheelsCmdSrvResponse()
        cmd_left = request.wheels_cmd.angular_velocities.joint[0]
        cmd_right = request.wheels_cmd.angular_velocities.joint[1]
        if cmd_left < -6 or cmd_left > 6 or cmd_right < -6 or cmd_right > 6:
            response.success = False
        else:
            self.cmd_wheels_msg.wheels_cmd = request.wheels_cmd
            response.success = True
        return response

    def stop_callback(self, request):
        """Callback function for the motors/stop service."""
        response = SetBoolResponse()
        if request.data:
            self.stop()
            rospy.sleep(0.5)
            response.success = True
            response.message = "Motors stopped"
        else:
            response.success = False
            response.message = "Enter 'true' to stop the motors"
        return response

    def stop(self):
        """Stop the motors."""
        time_relation = 1  # [sec]
        cmd_left = self.cmd_wheels_msg.wheels_cmd.angular_velocities.joint[0]
        cmd_right = self.cmd_wheels_msg.wheels_cmd.angular_velocities.joint[1]
        last_time = rospy.Time.now()
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        
        while dt < time_relation:
            dt = (current_time - last_time).to_sec()
            cmd_left = ((-cmd_left * dt) / time_relation) + cmd_left
            cmd_right = ((-cmd_right * dt) / time_relation) + cmd_right
            self.cmd_wheels_msg.wheels_cmd = WheelsCmd(AngularVelocities([cmd_left, cmd_right]))
            current_time = rospy.Time.now()

        self.cmd_wheels_msg.wheels_cmd = WheelsCmd(AngularVelocities([0, 0]))

    def odom_msg_init(self, v, w, odom_quat):
        """Initialize the Odometry message."""
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))
        self.odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))


if __name__ == '__main__':
    rospy.init_node('fellowme_controller_node', anonymous=True)
    fellow_ctrl = Controller()
    rate = rospy.Rate(30)
    
    while not fellow_ctrl.ctrl_c:
        fellow_ctrl.publish()
        rate.sleep()
