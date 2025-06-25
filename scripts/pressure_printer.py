#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
import sys
import csv
import os
import time

# ========== 사용자 설정 ==========
# ref_extended (rl_ref[:2] + ref[:6]) 에서 출력할 인덱스 리스트 (예: [2,3,4,5,6,7])
# selected_channels = [2, 3, 4, 5, 6, 7]
selected_channels = [2, 3, 5, 6]
# control 데이터 (micro, macro, atm)를 출력할지 여부
include_control_data = True
# ==================================

# 전역 데이터 변수
sen_values = []
ref_values = []
mpc_pwm_values = []
rl_pwm_values = []
rl_ref_values = []

# ANSI 코드 (화면 클리어 및 커서 이동)
CLEAR_SCREEN = "\033[2J"
MOVE_CURSOR_TO_TOP = "\033[H"

# CSV 파일 설정
csv_filename = "2DoF_Combined_test_v01_25_06_13.csv"
file_exists = os.path.isfile(csv_filename)
csv_file = open(csv_filename, mode='a', newline='')
csv_writer = csv.writer(csv_file)
if not file_exists:
    # CSV 헤더를 설정: 각 채널마다 ref, sen, error에 더해 control 데이터 포함 여부에 따라 컬럼 결정
    header = ["elapsed_ms"]
    for idx, ch in enumerate(selected_channels, start=1):
        header.extend([f"channel_{idx}_ref", f"channel_{idx}_sen", f"channel_{idx}_error"])
        if include_control_data:
            header.extend([f"channel_{idx}_micro", f"channel_{idx}_macro", f"channel_{idx}_atm"])
    csv_writer.writerow(header)

start_time = time.time()

# 콜백 함수 정의 (수신된 데이터를 전역 변수에 저장)
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

# 데이터 처리, CSV 저장 및 화면 출력 함수
def process_data():
    global sen_values, ref_values, mpc_pwm_values, rl_pwm_values, rl_ref_values, start_time, csv_writer, csv_file

    # 충분한 데이터가 수신된 경우에만 처리
    if len(sen_values) >= 9 and len(ref_values) >= 6:
        # 로컬 변수 캐싱
        sen = sen_values
        ref = ref_values
        rl_ref = rl_ref_values
        mpc_pwm = mpc_pwm_values

        # ref_extended는 rl_ref의 앞 2개와 ref의 앞 6개를 결합 (총 8개)
        ref_extended = rl_ref[:2] + ref[:6]

        elapsed_ms = int((time.time() - start_time) * 1000)
        row = [elapsed_ms]
        
        # 출력 문자열 구성: 리스트에 담고 join()을 사용
        output_lines = [
            MOVE_CURSOR_TO_TOP,
            "-----------"
        ]
        
        # 출력 헤더 문자열 구성
        header_cols = ["ref", "sen", "error"]
        if include_control_data:
            header_cols.extend(["micro", "macro", "atm"])
        output_lines.append(" ".join(f"{col:10}" for col in header_cols))
        
        # 각 선택 채널에 대해 데이터 처리
        # pwm 데이터는 선택된 채널 순서대로 (즉, 첫번째 selected_channels에 해당하는 control 데이터는 mpc_pwm[0:3], 두번째는 [3:6] 등)
        for idx, ch in enumerate(selected_channels):
            # ref 데이터: ref_extended[ch] (존재하지 않으면 0)
            ref_value = ref_extended[ch] if ch < len(ref_extended) else 0
            # sensor 데이터: sen[ch+1] (원래 코드에서 채널에 대해 offset +1 적용)
            sen_value = sen[ch+1] if (ch+1) < len(sen) else 0
            error = ref_value - sen_value
            
            if include_control_data:
                # PWM 데이터: 순서대로 3개씩 할당 (데이터 부족 시 0 할당)
                if len(mpc_pwm) >= (idx+1)*3:
                    micro = mpc_pwm[idx*3]
                    macro = mpc_pwm[idx*3+1]
                    atm = mpc_pwm[idx*3+2]
                else:
                    micro, macro, atm = (0, 0, 0)
                row.extend([ref_value, sen_value, error, micro, macro, atm])
                line = f"{ref_value:10.2f} {sen_value:10.2f} {error:10.2f} {micro:10} {macro:10} {atm:10}"
            else:
                row.extend([ref_value, sen_value, error])
                line = f"{ref_value:10.2f} {sen_value:10.2f} {error:10.2f}"
            
            output_lines.append(line)
        
        output_lines.append("-----------")
        output_str = "\n".join(output_lines) + "\n"
        
        # CSV 파일에 저장 및 flush
        csv_writer.writerow(row)
        csv_file.flush()

        # 화면 출력
        sys.stdout.write(CLEAR_SCREEN + output_str)
        sys.stdout.flush()

# 노드 종료 시 CSV 파일 닫기
def shutdown_hook():
    csv_file.close()

if __name__ == '__main__':
    rospy.init_node('sensor_ref_pair_printer', anonymous=True)

    # 토픽 구독
    rospy.Subscriber('/sen_values', Float32MultiArray, sen_values_callback)
    rospy.Subscriber('/mpc_ref_values', Float32MultiArray, ref_values_callback)
    rospy.Subscriber('/rl_ref_values', Float32MultiArray, rl_ref_values_callback)
    rospy.Subscriber('/raw_mpc_pwm', UInt16MultiArray, mpc_pwm_values_callback)
    rospy.Subscriber('/raw_rl_pwm', UInt16MultiArray, rl_pwm_values_callback)

    rospy.on_shutdown(shutdown_hook)
    
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        process_data()
        rate.sleep()
