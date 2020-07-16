#!/usr/bin/env python
from std_msgs.msg import Int32MultiArray
import rospy
import time
import Adafruit_ADS1x15
import random
from  math import pow, sqrt

class Calibra:
    def __init__(self):
        # Create the I2C bus
        self.adc = Adafruit_ADS1x15.ADS1015()
        self.pub = rospy.Publisher('command', Int32MultiArray, queue_size=1)
        array = []
        self.cmd = Int32MultiArray(data=array)
        self.cmd.data = [-1, -1, -1, -1, -1, -1, -1, -1 ,-1, -1, -1, -1, -1, -1, -1,-1]

    def calibra_s(self,puerto, angle_from, angle_to, sensor_pin, n_medidas):
        puntos_curva = n_medidas 
        paso = (angle_to - angle_from) / puntos_curva
        n_valores = 0
        x = []
        y = []
       
        voltage = self.mueve_servo(puerto, angle_to)
        voltage = self.mueve_servo(puerto, angle_from)

        rospy.loginfo("Midiendo Curva Servo %i en %i pasos", puerto, puntos_curva )
        
        for i in range(angle_from, angle_to, paso):
           # mover ese servo
           # rospy.loginfo("Publishing angle=%i" %i)
           voltage = self.mueve_servo(puerto, i)
           y.append(i)  #angle
           x.append(voltage)   #voltage
           n_valores = n_valores + 1
           # rospy.loginfo(" Puerto=%i  Voltage=%f5.3",puerto,voltage)
           rospy.sleep(1.5)
        m,n = self.simpLinReg(x,y, n_valores)
        n_medidas_error = 20
        suma_error2 = 0

        #for i in range(0,n_valores):
        #    print("'%i,%i'" % (x[i],y[i]))

        rospy.loginfo("Midiendo error con %i pruebas", n_medidas_error)
        for i in range(0, n_medidas_error):
           angulo_test = random.random()*180
           lectura_fback = self.mueve_servo(puerto,angulo_test)
           angulo_calculado = lectura_fback * m + n
           angulo_error2 = pow( angulo_test- angulo_calculado,2)
           print("fback=%i angulo_test=%i angulo_calculado=%i" % (lectura_fback,angulo_test, angulo_calculado))
           suma_error2 = suma_error2 + angulo_error2
        error_standard = sqrt( suma_error2 / (n_medidas_error-1) )
        return m, n, error_standard


    def mueve_servo(self, puerto, angulo):
        self.cmd.data[puerto] = self.degrees2control(angulo)
        self.pub.publish( self.cmd)
        rospy.sleep(1)
        self.cmd.data[puerto] = -1
        f = self.lee_feedback(puerto,1)
        return f

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

    def lee_feedback(self, sensor_pin, n_medidas):
        suma = 0
        promedio = 0

        for i in range(0, n_medidas):
            # time.sleep(0.1)
            s = self.adc.read_adc( sensor_pin, gain = 1 )
            print( "Lectura %i Sensor = %i" % (i,s) )
            suma = suma + s
        promedio = suma / n_medidas
        return( promedio)


    def simpLinReg(self, x, y, n):
        # pass x and y arrays (pointers), lrCoef pointer, and n.  
        # The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  
        #n is length of the x and y arrays.
        # http://en.wikipedia.org/wiki/Simple_linear_regression

        # initialize variables
        xbar=0.
        ybar=0.
        xybar=0.
        xsqbar=0.
  
        # calculations required for linear regression
        for i in range (0,n):
            xbar = xbar+x[i]
            ybar = ybar+y[i]
            xybar = xybar + x[i]*y[i]
            xsqbar = xsqbar + x[i]*x[i]
  
        xbar = xbar/n
        ybar = ybar/n
        xybar = xybar/n
        xsqbar = xsqbar/n
  
        
        # simple linear regression algorithm
        m = (xybar - xbar*ybar)/(xsqbar - xbar*xbar)
        n = ybar - m * xbar

        return m,n


if __name__ == '__main__':
    N_SERVOS = 2
    rospy.init_node('calibra_servo')
    o = Calibra()

    for i in range(0, N_SERVOS):
        rospy.loginfo("Servo %i  Puerto %i",i,i)
        m,n,error = o.calibra_s(puerto=i,angle_from=0,angle_to=180, sensor_pin=i,n_medidas=20)
        o.mueve_servo(i, 90)
        rospy.loginfo("m=%f  n=%f  error=%f",m,n,error)       
    rospy.loginfo("Ejecucion Finalizada")


