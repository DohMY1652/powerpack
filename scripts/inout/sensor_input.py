#!/usr/bin/env python3
import sys
import os
from collections import deque

# 상위 디렉토리의 High_Pricision_AD_HAT 폴더를 경로에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'High_Pricision_AD_HAT')))

import time
import ADS1263
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Float32MultiArray

REF = 5.08           # Reference voltage
N = 10               # Moving average 필터의 스텝 수
NUM_CHANNELS = 9     # 사용할 채널 수

ADC = ADS1263.ADS1263()

# 각 채널별 고정 크기의 버퍼와 running sum 미리 생성
buffers = [deque(maxlen=N) for _ in range(NUM_CHANNELS)]
running_sums = [0.0] * NUM_CHANNELS

def apply_moving_average_filter(channel_index, new_value):
    """
    moving average 필터를 running sum을 이용하여 최적화한 함수.
    버퍼가 가득 차면, 가장 오래된 값을 빼고 새 값을 더합니다.
    """
    buf = buffers[channel_index]
    if len(buf) == N:
        oldest = buf[0]
        buf.append(new_value)  # 새 값 추가 시 자동으로 oldest 제거
        running_sums[channel_index] = running_sums[channel_index] - oldest + new_value
    else:
        buf.append(new_value)
        running_sums[channel_index] += new_value
    return running_sums[channel_index] / len(buf)

def sensor_in():
    rospy.init_node('sensor_input', anonymous=True)
    pub = rospy.Publisher('sen_raw_values', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(50)  # 50Hz

    # 초기 ADC 설정
    if ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1:
        exit()
    ADC.ADS1263_SetMode(0)  # 0: singleChannel, 1: diffChannel
    channels = list(range(NUM_CHANNELS))  # 0~8번 채널

    # 상수 및 재사용할 변수들을 지역 변수에 저장
    ref = REF
    divisor_pos = 0x7fffffff
    divisor_neg = 0x80000000

    # 미리 할당된 메시지 객체와 필터링 결과 리스트 재사용
    msg = Float32MultiArray()
    filtered_values = [0.0] * NUM_CHANNELS

    while not rospy.is_shutdown():
        ADC_Value = ADC.ADS1263_GetAll(channels)  # ADC 값을 읽음 (9개 채널)

        for i, value in enumerate(ADC_Value):
            # 부호 확인 및 전압 값 변환
            if (value >> 31) == 1:
                data = ref * 2 - value * ref / divisor_neg
            else:
                data = value * ref / divisor_pos

            # moving average 필터 적용
            filtered_values[i] = apply_moving_average_filter(i, data)

        msg.data = filtered_values
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_in()
    except rospy.ROSInterruptException:
        pass
    finally:
        ADC.ADS1263_Exit()
