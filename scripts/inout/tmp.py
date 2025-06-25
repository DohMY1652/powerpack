#!/usr/bin/env python3

import rospy
import socket
import struct
import threading
from std_msgs.msg import Float32MultiArray

# 최신 데이터를 저장할 전역 변수와 스레드 동기화를 위한 락
latest_data = None
data_lock = threading.Lock()

def data_receiver(client_socket):
    """UDP로 데이터를 수신하여 전역 변수에 저장."""
    global latest_data
    message_size = 6 * 4  # 6개의 float, 총 24바이트

    while not rospy.is_shutdown():
        try:
            data, addr = client_socket.recvfrom(24)
            if not data:
                rospy.logwarn("No data received.")
                continue
            if len(data) < message_size:
                rospy.logwarn(f"Incomplete UDP packet received: {len(data)} bytes")
                continue

            message_data = data[:message_size]
            values = struct.unpack('f' * 6, message_data)
            # 예제에서는 각 값에 101.325를 더해줍니다.
            values = [v + 101.325 for v in values]

            with data_lock:
                latest_data = values

            rospy.loginfo(f"Received data: {values}")

        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            break
        except struct.error as e:
            rospy.logerr(f"Struct unpacking error: {e}")
            break

    client_socket.close()
    rospy.loginfo("UDP socket closed.")

def udp_node():
    global latest_data
    rospy.init_node('udp_to_ros', anonymous=True)
    pub = rospy.Publisher('/mpc_ref_values', Float32MultiArray, queue_size=1)

    #  ROBOTORY IP : 169.254.122.65
    HOST = '0.0.0.0'  # 모든 인터페이스에서 수신
    PORT = 8688

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.bind((HOST, PORT))
    rospy.loginfo(f"Listening for UDP packets on {HOST}:{PORT}")

    # 별도 스레드에서 UDP 데이터를 계속 수신
    receiver_thread = threading.Thread(target=data_receiver, args=(client_socket,), daemon=True)
    receiver_thread.start()

    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        with data_lock:
            if latest_data is not None:
                msg = Float32MultiArray()
                msg.data = latest_data
                pub.publish(msg)
                rospy.loginfo(f"Published: {latest_data}")
        rate.sleep()

    client_socket.close()
    rospy.loginfo("UDP node shut down.")

if __name__ == '__main__':
    try:
        udp_node()
    except rospy.ROSInterruptException:
        pass
