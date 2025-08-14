#!/usr/bin/env python3

import rospy
import socket
import threading
from std_msgs.msg import Float32MultiArray

latest_data = None
data_lock = threading.Lock()

def data_receiver(tcp_socket):
    global latest_data
    channel = 8
    rx_bytes = channel * 2  # uint16 * 6 = 12 bytes
    buffer = b''

    tcp_socket.settimeout(1.0)  # ⏱ 1초 timeout 설정

    while not rospy.is_shutdown():
        try:
            while len(buffer) < rx_bytes:
                try:
                    chunk = tcp_socket.recv(rx_bytes - len(buffer))
                    if not chunk:
                        rospy.logwarn("TCP connection closed by remote host.")
                        return
                    buffer += chunk
                except socket.timeout:
                    rospy.logwarn("Timed out waiting for data.")
                    break  # 루프 빠져나가서 다음 recv로

            if len(buffer) < rx_bytes:
                continue  # 다음 루프에서 다시 recv 시도

            message = buffer[:rx_bytes]
            buffer = buffer[rx_bytes:]

            echoed_vals = []
            for k in range(channel):
                high = message[2 * k]
                low = message[2 * k + 1]
                value = (high << 8) | low
                echoed_vals.append(float(value) / 100.0)

            with data_lock:
                latest_data = echoed_vals

            # rospy.loginfo(f"Received TCP data: {echoed_vals}")

        except socket.error as e:
            rospy.logerr(f"TCP socket error: {e}")
            break

    tcp_socket.close()
    rospy.loginfo("TCP socket closed.")

def tcp_node():
    global latest_data
    rospy.init_node('tcp_to_ros', anonymous=True)
    pub = rospy.Publisher('/mpc_ref_values', Float32MultiArray, queue_size=1)

    HOST = '169.254.46.254'
    PORT = 2265

    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    rospy.loginfo(f"Connecting to TCP server at {HOST}:{PORT} ...")

    try:
        tcp_socket.connect((HOST, PORT))
        rospy.loginfo("Connected to LabVIEW TCP server.")
    except Exception as e:
        rospy.logerr(f"Connection failed: {e}")
        return

    receiver_thread = threading.Thread(target=data_receiver, args=(tcp_socket,), daemon=True)
    receiver_thread.start()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        with data_lock:
            if latest_data is not None:
                msg = Float32MultiArray()
                msg.data = latest_data
                pub.publish(msg)
                # rospy.loginfo(f"Published: {latest_data}")
        rate.sleep()

    tcp_socket.close()
    rospy.loginfo("TCP node shut down.")


if __name__ == '__main__':
    try:
        tcp_node()
    except rospy.ROSInterruptException:
        pass
