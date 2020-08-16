#!/usr/bin/env python
from std_msgs.msg import Int32MultiArray
import rospy
import time
import Adafruit_ADS1x15
import random
from  math import pow, sqrt
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Este proceso publica el estado de los sensores

class Publisher:
    def __init__(self):
        # Create the I2C bus
        self.adc = Adafruit_ADS1x15.ADS1015()
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher')
        self.rate = rospy.Rate(10) # 10Hz
        self.data = JointState()
        self.data.header = Header()
        self.data.name = ['link_01_name__link_02_name', 'link_02_name__link_03_name','not_used_3','not_used_4']
        self.data.velocity = []
        self.data.effort = []
        self.data.position = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]

    def talker(self):
        while not rospy.is_shutdown():
            self.data.header.stamp = rospy.Time.now()
            for i in range(0,4):
                self.data.position[i] = self.lee_feedback(i,1)
            self.pub.publish(self.data)
            self.rate.sleep()

    def lee_feedback(self, sensor_pin, n_medidas):
        suma = 0
        promedio = 0

        for i in range(0, n_medidas):
            # time.sleep(0.1)
            s = self.adc.read_adc( sensor_pin, gain = 1 )
            # print( "Lectura %i Sensor = %i" % (i,s) )
            suma = suma + s
        promedio = suma / n_medidas
        return( promedio)


if __name__ == '__main__':
    p = Publisher()
    try:
        p.talker()
    except rospy.ROSInterruptException:
        pass

