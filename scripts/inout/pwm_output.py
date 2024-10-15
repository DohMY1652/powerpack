#!/usr/bin/env python

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'Adafruit_Python_PCA9685')))

import rospy
from std_msgs.msg import UInt16MultiArray
import time
import Adafruit_PCA9685

pwm_module_1 = Adafruit_PCA9685.PCA9685(0x40)
pwm_module_2 = Adafruit_PCA9685.PCA9685(0x41)


valveMin = 0
valveMax = 4095

def map(value, close_length, open_length,min_pulse, max_pulse):
    open_range = open_length-close_length
    pulse_range = max_pulse-min_pulse
    scale_factor = float(open_range)/float(pulse_range)

    return min_pulse + (value/scale_factor)

def set_value(module, channel,value):

    pulse = int(map(value,0, 1000, valveMin,valveMax))
    module.set_pwm(channel,0,pulse)

pwm_module_1.set_pwm_freq(1000)
pwm_module_2.set_pwm_freq(1000)


def callback(data):
    uint16_data = data.data
    if len(uint16_data) != 32:
        rospy.loginfo("Not enough amount of data. please check")
    else:
        rospy.loginfo("Received data: %s", uint16_data)
        rospy.loginfo("Length of received data: %s", len(uint16_data))
        i = 0
        for pwm in uint16_data:
            if i <= 15:
                set_value(pwm_module_1,i, pwm)
            else :
                set_value(pwm_module_2,i-16, pwm)
            i += 1
                
            
def control_out():
    rospy.init_node('pwm_output', anonymous=True)
    rospy.Subscriber('pwm_values', UInt16MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    control_out()
