#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
from collections import deque

class OutputIntegrator:
    # Set N values as class variables at the top
    N_mpc = 1
    N_rl = 1

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('output_integrator', anonymous=True)

        # Create Publishers
        self.pub = rospy.Publisher('pwm_values', UInt16MultiArray, queue_size=1)

        # Create Subscribers
        self.sub_mpc = rospy.Subscriber('mpc_pwm', UInt16MultiArray, self.callback_mpc, queue_size=1)
        self.sub_rl = rospy.Subscriber('rl_pwm', UInt16MultiArray, self.callback_rl, queue_size=1)

        # Set up a timer to publish periodically
        self.rate = rospy.Rate(50)

        # Separate data buffers for mpc and rl
        self.data_mpc = deque(maxlen=self.N_mpc)
        self.data_rl = deque(maxlen=self.N_rl)

    def callback_mpc(self, msg):
        self.data_mpc.append(msg.data)
        rospy.loginfo("Received from mpc_pwm: %s", msg.data)
        self.merge_and_publish()

    def callback_rl(self, msg):
        self.data_rl.append(msg.data)
        rospy.loginfo("Received from rl_pwm: %s", msg.data)
        self.merge_and_publish()

    def apply_moving_average(self, data_deque, N):
        # Calculate the moving average for each element across N data points
        if len(data_deque) < N:
            return data_deque[-1]  # Not enough data yet, return the latest one

        avg_data = [sum(x[i] for x in data_deque) // len(data_deque) for i in range(len(data_deque[0]))]
        return avg_data

    def merge_and_publish(self):
        if len(self.data_mpc) == self.N_mpc and len(self.data_rl) == self.N_rl:
            # Apply moving average filter
            averaged_mpc = self.apply_moving_average(self.data_mpc, self.N_mpc)
            averaged_rl = self.apply_moving_average(self.data_rl, self.N_rl)

            merged_data = averaged_mpc[0:12] + [0]*4 + averaged_mpc[12:24] + averaged_rl + [0]*2  # Merge arrays by concatenation
            output_msg = UInt16MultiArray()
            output_msg.data = merged_data

            self.pub.publish(output_msg)
            rospy.loginfo("Published merged array: %s", output_msg.data)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    node = OutputIntegrator()  # N_mpc and N_rl are set as class variables
    node.run()
