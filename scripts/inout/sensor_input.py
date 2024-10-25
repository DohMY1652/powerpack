import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'High_Pricision_AD_HAT')))

import time
import ADS1263
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Float32MultiArray

REF = 5.08  
ADC = ADS1263.ADS1263()
adc_values = [0, 0, 0, 0, 0, 0, 0, 0, 0]
def sensor_in():
    rospy.init_node('sensor_input', anonymous=True)
    pub = rospy.Publisher('sen_values', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(500)  # 500 Hz

    if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
        exit()
    ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel
    channelList = [0, 1, 2, 3, 4, 5, 6, 7, 8]  # The channel must be less than 10
    while not rospy.is_shutdown():
        os.system('clear')  
        ADC_Value = ADC.ADS1263_GetAll(channelList)    # get ADC1 value
        for i in channelList:
            if(ADC_Value[i]>>31 ==1):
                data = (REF*2 - ADC_Value[i] * REF / 0x80000000)
                rospy.loginfo("ADC1 IN%d = -%lf" %(i, data))  
            else:
                data = (ADC_Value[i] * REF / 0x7fffffff)
                rospy.loginfo("ADC1 IN%d = %lf" %(i, data))   # 32bit
            adc_values[i] = data
            
        msg = Float32MultiArray(data=adc_values)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_in()
    except rospy.ROSInterruptException:
        pass

    ADC.ADS1263_Exit()