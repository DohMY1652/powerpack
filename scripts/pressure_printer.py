#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
import sys

# 전역 변수 초기화
sen_values = []
ref_values = []
mpc_pwm_values = []
rl_pwm_values = []
rl_ref_values = []

# ANSI 코드로 화면 지우기 및 커서 이동
CLEAR_SCREEN = "\033[2J"
MOVE_CURSOR_TO_TOP = "\033[H"

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

# 데이터 처리 및 출력 함수
def process_data():
    if len(sen_values) >= 9 and len(ref_values) >= 6:
        ref_values_extended = rl_ref_values[:2] + ref_values[:6]
        selected_pairs = [2, 3, 4, 5, 6, 7]

        output = MOVE_CURSOR_TO_TOP  # 커서를 화면 상단으로 이동

        # 첫 번째 테이블 출력
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

            output += f"{ref_value:10.2f} {sen_value:10.2f} {error:10.2f} {micro:10} {macro:10} {atm:10}\n"

        # 두 번째 테이블 출력
        output += "===========\n"
        output += f"{'ref':10} {'sen':10} {'error':10} {'pwm':10} {'gauge pressure':10}\n"

        if len(rl_pwm_values) == 2:
            pwm_value_6, pwm_value_7 = rl_pwm_values

            ref_value_6 = ref_values_extended[0] - 101.325 if len(ref_values_extended) > 6 else 0
            sen_value_6 = sen_values[0] - 101.325 if len(sen_values) > 0 else 0
            error_6 = ref_value_6 - sen_value_6

            ref_value_7 = ref_values_extended[1] - 101.325 if len(ref_values_extended) > 7 else 0
            sen_value_7 = sen_values[1] - 101.325 if len(sen_values) > 1 else 0
            error_7 = -1 * (ref_value_7 - sen_value_7)

            output += f"{ref_value_6:10.2f} {sen_value_6:10.2f} {error_6:10.2f} {pwm_value_6:10}\n"
            output += f"{ref_value_7:10.2f} {sen_value_7:10.2f} {error_7:10.2f} {pwm_value_7:10}\n"

        # 두 번째 테이블 출력
        output += "===========\n"

        sen_value_macro = sen_values[2]
        
        output += f"{'Macro pressure':10}\n"
        output += f"{sen_value_macro:10.2f}\n"

        output += "-----------\n"

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
