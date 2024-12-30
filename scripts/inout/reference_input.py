#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32MultiArray

def serial_node():
    # ROS 노드 초기화
    rospy.init_node('serial_to_ros', anonymous=True)
    pub = rospy.Publisher('/ref_values', Float32MultiArray, queue_size=10)

    # 시리얼 포트 설정 (포트 이름과 속도 확인 필요)
    port = '/dev/ttyACM0'  # 아두이노가 연결된 포트
    baud_rate = 115200
    ser = serial.Serial(port, baud_rate, timeout=1)

    rospy.loginfo("Serial port initialized. Waiting for data...")

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                values = [float(x) for x in line.split(',')]  # 콤마로 분리 후 변환
                if len(values) == 6:
                    msg = Float32MultiArray()
                    msg.data = values
                    pub.publish(msg)
                    rospy.loginfo(f"Published: {values}")
            except ValueError as e:
                rospy.logwarn(f"Failed to parse data: {e}")

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
