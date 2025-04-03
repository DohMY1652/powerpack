#!/usr/bin/env python3

import rospy
import socket
import struct
import threading
from std_msgs.msg import Float32MultiArray

def data_receiver(pub, client_socket):
    """UDP로 데이터를 수신하고 ROS 토픽으로 Publish."""
    message_size = 6 * 4  # 6개의 float, 총 24바이트

    while not rospy.is_shutdown():
        try:
            data, addr = client_socket.recvfrom(24)  # 최대 1024바이트 수신
            if not data:
                rospy.logwarn("No data received.")
                continue

            if len(data) < message_size:
                rospy.logwarn(f"Incomplete UDP packet received: {len(data)} bytes")
                continue

            # 필요한 만큼만 사용 (나머지는 무시)
            message_data = data[:message_size]
            values = struct.unpack('f' * 6, message_data)
            values = [v + 101.325 for v in values]

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

    client_socket.close()
    rospy.loginfo("UDP socket closed.")

def udp_node():
    rospy.init_node('udp_to_ros', anonymous=True)
    pub = rospy.Publisher('/mpc_ref_values', Float32MultiArray, queue_size=1)

    HOST = '0.0.0.0'  # 모든 인터페이스에서 수신
    PORT = 8688

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.bind((HOST, PORT))
    rospy.loginfo(f"Listening for UDP packets on {HOST}:{PORT}")

    receiver_thread = threading.Thread(
        target=data_receiver,
        args=(pub, client_socket),
        daemon=True
    )
    receiver_thread.start()

    rospy.spin()

    client_socket.close()
    rospy.loginfo("UDP node shut down.")

if __name__ == '__main__':
    try:
        udp_node()
    except rospy.ROSInterruptException:
        pass
