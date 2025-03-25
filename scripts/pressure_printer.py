#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
import sys
import csv
import os
import time

# 전역 변수 초기화
sen_values = []
ref_values = []
mpc_pwm_values = []
rl_pwm_values = []
rl_ref_values = []

# ANSI 코드로 화면 지우기 및 커서 이동
CLEAR_SCREEN = "\033[2J"
MOVE_CURSOR_TO_TOP = "\033[H"

# CSV 파일 설정
csv_filename = "arm_test_10kPa_diff.csv"
file_exists = os.path.isfile(csv_filename)

# CSV 파일 열기 및 헤더 작성 (처음 실행 시에만)
with open(csv_filename, mode='a', newline='') as file:
    writer = csv.writer(file)
    if not file_exists:
        writer.writerow(["elapsed_ms", "channel_1_ref", "channel_1_sen", "channel_1_error", "channel_1_micro", "channel_1_macro", "channel_1_atm",
                         "channel_2_ref", "channel_2_sen", "channel_2_error", "channel_2_micro", "channel_2_macro", "channel_2_atm",
                         "channel_3_ref", "channel_3_sen", "channel_3_error", "channel_3_micro", "channel_3_macro", "channel_3_atm",
                         "channel_4_ref", "channel_4_sen", "channel_4_error", "channel_4_micro", "channel_4_macro", "channel_4_atm",
                         "channel_5_ref", "channel_5_sen", "channel_5_error", "channel_5_micro", "channel_5_macro", "channel_5_atm",
                         "channel_6_ref", "channel_6_sen", "channel_6_error", "channel_6_micro", "channel_6_macro", "channel_6_atm"])

start_time = time.time()

# 콜백 함수 정의
def sen_values_callback(msg):
    global sen_values
    sen_values = list(msg.data)

def ref_values_callback(msg):
    global ref_values
    ref_values = list(msg.data)

def rl_ref_values_callback(msg):
    global rl_ref_values
    rl_ref_values = list(msg.data)

def mpc_pwm_values_callback(msg):
    global mpc_pwm_values
    mpc_pwm_values = list(msg.data)

def rl_pwm_values_callback(msg):
    global rl_pwm_values
    rl_pwm_values = list(msg.data)

# 데이터 처리 및 저장 함수
def process_data():
    if len(sen_values) >= 9 and len(ref_values) >= 6:
        ref_values_extended = rl_ref_values[:2] + ref_values[:6]
        selected_pairs = [2, 3, 4, 5, 6, 7]

        elapsed_ms = int((time.time() - start_time) * 1000)
        row = [elapsed_ms]
        
        output = MOVE_CURSOR_TO_TOP
        output += "-----------\n"
        output += f"{'ref':10} {'sen':10} {'error':10} {'micro':10} {'macro':10} {'atm':10}\n"

        for i in selected_pairs:
            ref_value = ref_values_extended[i] if i < len(ref_values_extended) else 0
            sen_value = sen_values[i+1] if i+1 < len(sen_values) else 0
            error = ref_value - sen_value

            micro, macro, atm = (0, 0, 0)
            if len(mpc_pwm_values) >= ((i - 2) + 1) * 3:
                micro = mpc_pwm_values[(i - 2) * 3]
                macro = mpc_pwm_values[(i - 2) * 3 + 1]
                atm = mpc_pwm_values[(i - 2) * 3 + 2]
            
            row.extend([ref_value, sen_value, error, micro, macro, atm])
            output += f"{ref_value:10.2f} {sen_value:10.2f} {error:10.2f} {micro:10} {macro:10} {atm:10}\n"
        
        output += "-----------\n"

        # CSV 파일에 저장
        with open(csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(row)

        # 화면에 출력
        sys.stdout.write(CLEAR_SCREEN + output)
        sys.stdout.flush()

# 메인 함수
if __name__ == '__main__':
    rospy.init_node('sensor_ref_pair_printer', anonymous=True)

    # 토픽 구독
    rospy.Subscriber('/sen_values', Float32MultiArray, sen_values_callback)
    rospy.Subscriber('/mpc_ref_values', Float32MultiArray, ref_values_callback)
    rospy.Subscriber('/rl_ref_values', Float32MultiArray, rl_ref_values_callback)
    rospy.Subscriber('/raw_mpc_pwm', UInt16MultiArray, mpc_pwm_values_callback)
    rospy.Subscriber('/raw_rl_pwm', UInt16MultiArray, rl_pwm_values_callback)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        process_data()
        rate.sleep()
