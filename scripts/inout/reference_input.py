#!/usr/bin/env python3

import rospy
import socket
import struct
import threading
from std_msgs.msg import Float32MultiArray

def data_receiver(pub, client_socket):
    """별도의 스레드에서 TCP 소켓 데이터를 수신하고, 토픽으로 Publish."""
    buffer = b''
    message_size = 6 * 4  # 6개의 float, 총 24바이트

    while not rospy.is_shutdown():
        try:
            data = client_socket.recv(24)  # 24바이트만큼 수신
            if not data:
                rospy.logwarn("Server closed connection.")
                break

            buffer += data

            # 버퍼에 24바이트(=6 float) 이상 있으면 패킷 단위로 파싱
            while len(buffer) >= message_size:
                message_data = buffer[:message_size]
                buffer = buffer[message_size:]

                values = struct.unpack('f' * 6, message_data)
                # 6개의 모든 reference 값에 101.325를 더함
                values = [v + 101.325 for v in values]

                # ROS 메시지 발행
                msg = Float32MultiArray()
                msg.data = values
                pub.publish(msg)
                rospy.loginfo(f"Published: {values}")

        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            break
        except struct.error as e:
            rospy.logerr(f"Struct unpacking error: {e}")
            break

    # 루프 종료 시 소켓 닫기
    client_socket.close()
    rospy.loginfo("Data receiver thread ended.")

def tcp_node():
    rospy.init_node('tcp_to_ros', anonymous=True)
    pub = rospy.Publisher('/mpc_ref_values', Float32MultiArray, queue_size=1)

    HOST = '192.168.3.10'
    # HOST = '192.168.0.40'
    PORT = 8688

    # TCP 클라이언트 소켓 생성
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Nagle 알고리즘 끄기(지연 최소화)
    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    try:
        client_socket.connect((HOST, PORT))
        rospy.loginfo(f"Connected to server {HOST}:{PORT}")
    except Exception as e:
        rospy.logerr(f"Connection failed: {e}")
        return

    # 별도 스레드에서 데이터 수신 전담
    receiver_thread = threading.Thread(
        target=data_receiver, 
        args=(pub, client_socket), 
        daemon=True
    )
    receiver_thread.start()

    # 메인 스레드는 ROS 이벤트 루프(콜백 처리 등) 담당
    rospy.spin()

    # 노드 종료 시 소켓 닫기 (스레드도 함께 종료될 것)
    client_socket.close()
    rospy.loginfo("TCP connection closed.")

if __name__ == '__main__':
    try:
        tcp_node()
    except rospy.ROSInterruptException:
        pass
