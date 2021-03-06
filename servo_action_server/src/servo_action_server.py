#! /usr/bin/env python
import rospy
import actionlib
import RPi.GPIO as GPIO
from std_msgs.msg import Int32MultiArray

#from adafruit_servokit import ServoKit
import time
#el nombre del import viene del nombre del archivo que se deja en el foldel action del pkg servo_msgs 
from servo_msgs.msg import ServoActionMsgFeedback
from servo_msgs.msg import ServoActionMsgResult
from servo_msgs.msg import ServoActionMsgAction
# goal
#   int8 servo_num
#   float32 goal_angle
#   float32 step_angle
# result
#   bool success
# feedback
#   float32 current_angle

#using servos  HJ S3315D  Range could change with other servos, be careful. you can start with 1000,2000

#kit = ServoKit(channels=16)
#kit.servo[0].set_pulse_width_range(500,2500)
#kit.servo[1].set_pulse_width_range(500,2500)

def degrees2control(angle):
    #constants
    f = 50 #Hz
    min_pwm = 500  #0 degrees
    max_pwm = 2500 #180 degrees

    #calculations
    m = (max_pwm - min_pwm) / (180-0)
    n = max_pwm - m*180
    pulse = m*angle + n  #micro seconds
    control = pulse * f * 65536 / 1000000
    return int(control)

class ServoClass(object):
    # create messagesthat are used to publish feedback/result
    # rosmsg list | grep plotter2dof
    _feedback = ServoActionMsgFeedback()
    _result   = ServoActionMsgResult()

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("server_as", ServoActionMsgAction, self.goal_callback, False)
        self._as.start()

    def goal_callback(self, goal):
        # this callback is called when the action server is called
        # this is the function that execute the server actions
        # and returns the server status to the node that called the action server

        # helper variables
        r = rospy.Rate(1)
        success = True

        #define the different publishers and messages that ill be used
        #rospy.init_node('topic_publisher')
        self._pub_command = rospy.Publisher('/command', Int32MultiArray, queue_size=1)
        self._command = Int32MultiArray()
        rate = rospy.Rate(2)
        self._command.data = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]



        #Starts with the movement
        rospy.loginfo('"server_as": Executing, moving SERVO to %i degrees' % (goal.goal_angle))

        #starts moving to the origin
        #kit.servo[0].angle = 0 
        self._command.data[0] = degrees2control(0)
        self._pub_command.publish(self._command)
        
        for angle in range(0, int(goal.goal_angle),1):
           # check that preempt (cancelation) has not been requested by the action client
           if self._as.is_preempt_requested():
               respy.loginfo('The golad has been cancelled/preempted')
               # the following line, sets the client in preemted state (goal cancelled)
               self._as.set_preemted()
               success = False
               # we end the movement of the server
               break

           self._command.data[0] = degrees2control(angle)
           self._pub_command.publish(self._command)
           #builds the next feedback msg to be send
           self._as.publish_feedback( self._feedback)
           # the movement is computed at 1Hz frequency
           r.sleep()

        # at this point, either the goal has been achieved (success == true)
        # or the client preemted the goad (success=false)
        # if success, then we publish the final result
        # if not success, we do not publish anything in the result
        if success:
           self._result.success = success 
           rospy.loginfo('Succeeded moving the servo to %i' %goal.goal_angle)
           self._as.set_succeeded(self._result)

if __name__ == '__main__':
   rospy.init_node('servo')
   ServoClass()
   rospy.spin()


