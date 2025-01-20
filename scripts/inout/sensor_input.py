import sys
import os
from collections import deque

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'High_Pricision_AD_HAT')))

import time
import ADS1263
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Float32MultiArray

REF = 5.08  # Reference voltage
ADC = ADS1263.ADS1263()
N = 10  # Number of steps for the moving average filter
adc_values = [0] * 9
filtered_values = [0] * 9
buffers = [deque(maxlen=N) for _ in range(9)]  # Create a buffer for each channel

def apply_moving_average_filter(channel_index, new_value):
    """Apply moving average filter to the given channel."""
    buffer = buffers[channel_index]
    buffer.append(new_value)
    return sum(buffer) / len(buffer)

def sensor_in():
    rospy.init_node('sensor_input', anonymous=True)
    pub = rospy.Publisher('sen_raw_values', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(1000)  # 1 kHz

    if ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1:
        exit()
    ADC.ADS1263_SetMode(0)  # 0 is singleChannel, 1 is diffChannel
    channel_list = [0, 1, 2, 3, 4, 5, 6, 7, 8]  # Channels to read

    while not rospy.is_shutdown():
        ADC_Value = ADC.ADS1263_GetAll(channel_list)  # Get ADC1 value
        for i in channel_list:
            if ADC_Value[i] >> 31 == 1:
                data = REF * 2 - ADC_Value[i] * REF / 0x80000000
            else:
                data = ADC_Value[i] * REF / 0x7fffffff
            
            adc_values[i] = data
            filtered_values[i] = apply_moving_average_filter(i, data)

        msg = Float32MultiArray(data=filtered_values)
        rospy.loginfo(f"Filtered values: {filtered_values}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_in()
    except rospy.ROSInterruptException:
        pass
    finally:
        ADC.ADS1263_Exit()
