#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import Float32MultiArray

def tcp_node():
    # ROS 노드 초기화
    rospy.init_node('tcp_to_ros', anonymous=True)
    pub = rospy.Publisher('/mpc_ref_values', Float32MultiArray, queue_size=10)

    # 연결할 서버의 IP와 포트 (예시)
    HOST = '192.168.0.40'
    PORT = 8688

    # TCP 클라이언트 소켓 생성 및 서버 연결
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    rospy.loginfo(f"Connected to server {HOST}:{PORT}")

    # 발행 주기 (1kHz)
    rate = rospy.Rate(1000)

    # 수신 데이터를 임시로 보관할 버퍼
    buffer = ""

    while not rospy.is_shutdown():
        try:
            # 서버로부터 데이터 수신 (1024바이트씩 읽기)
            data = client_socket.recv(1024)
            # 빈 데이터가 오면 서버가 연결을 끊은 것으로 간주
            if not data:
                rospy.logwarn("Server closed connection.")
                break

            # 버퍼에 누적 후, 줄바꿈(\n) 기준으로 분할
            buffer += data.decode('utf-8')
            lines = buffer.split('\n')
            # 마지막 줄은 아직 완전히 수신되지 않았을 수 있으므로 buffer에 남겨둠
            buffer = lines[-1]

            # 완성된 라인(마지막을 제외한 나머지)들을 처리
            for line in lines[:-1]:
                line = line.strip()
                if not line:
                    continue  # 빈 줄은 무시

                # 콤마로 구분된 float 6개 파싱
                try:
                    values = [float(x) for x in line.split(',')]
                    if len(values) == 6:
                        # ROS 메시지 발행
                        msg = Float32MultiArray()
                        msg.data = values
                        pub.publish(msg)
                        rospy.loginfo(f"Published: {values}")
                    else:
                        rospy.logwarn(f"Received unexpected number of values: {len(values)}. Data: {line}")
                except ValueError as e:
                    rospy.logwarn(f"Failed to parse data: {e} / Raw data: {line}")

        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            break

        # 1kHz 주기로 루프 실행
        rate.sleep()

    # 노드 종료 시 소켓 닫기
    client_socket.close()
    rospy.loginfo("TCP connection closed.")

if __name__ == '__main__':
    try:
        tcp_node()
    except rospy.ROSInterruptException:
        pass
