#!/usr/bin/env python3

# import libraries & ros plugins & msgs
import numpy as np
import rospy
from math import pi , cos , sin , asin
from std_msgs.msg import Float64 ,Float32 , Int16
from fellowme_msgs.srv import PwmVal , PwmValResponse
from std_srvs.srv import SetBool , SetBoolResponse
import time



class Controller:

    def __init__(self):
        
        ## init publisher and subscribers
        self.left_pwm_publisher = rospy.Publisher("/left_motor_pwm",Int16,queue_size=100) 
        self.right_pwm_publisher = rospy.Publisher("/right_motor_pwm",Int16,queue_size=100) 
        # self.left_encoder_sub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        # self.right_encoder_sub = rospy.Subscriber("/encoder_right_ticks",Int16,self.rightEncoder_callback)


        self.t_0 = time.time()

        ## init services
        self.set_pwm_service = rospy.Service('motors/set_pwm', PwmVal , self.set_pwm_callback)
        self.motors_stop_service = rospy.Service('motors/stop', SetBool , self.stop_callback)

        ## init time variables 
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        ## init msgs ##
        ## left params
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 0        


        ## right params
        self.pwm_right_out =  Int16()
        self.pwm_right_out.data = 0      


        # init kinematics parameters
        self.R = 0.1016/2    
        self.L = 0.296       # need to update


        ## shutdownhook process
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        
       # self.update_pose()

    def shutdownhook(self):
        rospy.loginfo("shutting down")
        self.ctrl_c = True
        self.stop()
        
        
            
    def publish(self):
        
        # publish pwm
        self.left_pwm_publisher.publish(self.pwm_left_out)
        self.right_pwm_publisher.publish(self.pwm_right_out)

    def set_pwm_callback(self, request):
        response = PwmValResponse()
        if ((request.pwm_left<-250)or(request.pwm_left>250)or(request.pwm_right<-250)or(request.pwm_right>250)):
            response.success = False
        else:
            self.pwm_left_out.data = request.pwm_left
            self.pwm_right_out.data = request.pwm_right
            response.success = True
        return response

    def stop_callback(self, request):
        
        response = SetBoolResponse()
        if request.data:
            self.stop()
            rospy.sleep(0.5)
            response.success = True
            response.message = "motors stopped"

        else:
            response.success = False
            response.message = "enter 'true' to stop the motors" 

        return response

    def stop(self):
        time_relation = 1 # [sec]
        pwm_left = self.pwm_left_out.data
        pwm_right = self.pwm_right_out.data
        last_time = rospy.Time.now()
        current_time = rospy.Time.now()
        dt  = (current_time - last_time).to_sec()
        while(dt < time_relation):  ### need to publish pwm? 
            dt  = (current_time - last_time).to_sec()
            self.pwm_left_out.data = int(((-pwm_left*dt)/time_relation)) + pwm_left
            self.pwm_right_out.data = int(((-pwm_right*dt)/time_relation)) + pwm_right    
            current_time = rospy.Time.now()

        self.pwm_left_out.data = 0
        self.pwm_right_out.data = 0


if __name__ == '__main__':
    rospy.init_node('fellowme_controller_node', anonymous=True)    
    fellow_ctrl = Controller()
    rate = rospy.Rate(30)
    while not fellow_ctrl.ctrl_c:
       fellow_ctrl.publish()
       rate.sleep()
    