#!/usr/bin/env python3

import rospy
import smach 
from smach import CBState
import smach_ros
import smach_msgs

# define state follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        pass

    def execute(self, userdata):
        rospy.loginfo('Executing state FOLLOW')


# define state servo
class Servo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
        pass

    def execute(self, userdata):
        rospy.loginfo('Executing state SERVO')



# # First you create a state machine sm
# # .....
# # Creating of state machine sm finished

# # Create and start the introspection server
# sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
# sis.start()

# # Execute the state machine
# outcome = sm.execute()

# # Wait for ctrl-c to stop the application
# rospy.spin()
# sis.stop()
