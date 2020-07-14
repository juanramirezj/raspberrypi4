#!/usr/bin/env python
from std_msgs.msg import Int32MultiArray
import rospy
import time
import Adafruit_ADS1x15

class Calibra:
    def __init__(self):
        # Create the I2C bus
        self.adc = Adafruit_ADS1x15.ADS1015()
        self.pub = rospy.Publisher('command', Int32MultiArray, queue_size=1)
        array = []
        self.cmd = Int32MultiArray(data=array)
        self.cmd.data = [-1, -1, -1, -1, -1, -1, -1, -1 ,-1, -1, -1, -1, -1, -1, -1,-1]

    def calibra_s(self,puerto, angle_from, angle_to, sensor_pin, n_medidas, coeficientes):
        paso = 1
        for i in range(angle_from, angle_to, paso):
           # mover ese servo
           rospy.loginfo("Publishing angle=%i" %i)
           self.cmd.data[puerto] = self.degrees2control(i)
           self.pub.publish( self.cmd )
           rospy.sleep(1)


    def degrees2control(self,angle):
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


if __name__ == '__main__':
    rospy.init_node('calibra_servo')
    o = Calibra()
    o.calibra_s(0,0,100,1,1,1)

