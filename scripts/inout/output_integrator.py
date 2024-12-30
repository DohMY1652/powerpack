#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray

class OutputIntegrator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('output_integrator', anonymous=True)

        # Create Publishers
        self.pub = rospy.Publisher('pwm_values', UInt16MultiArray, queue_size=1)

        # Create Subscribers
        self.sub_mpc = rospy.Subscriber('mpc_pwm', UInt16MultiArray, self.callback_mpc)
        self.sub_rl = rospy.Subscriber('rl_pwm', UInt16MultiArray, self.callback_rl)

        # Set up a timer to publish periodically
        self.rate = rospy.Rate(100)  # 100 Hz

        self.data_mpc = None
        self.data_rl = None

    def callback_mpc(self, msg):
        self.data_mpc = msg.data
        rospy.loginfo("Received from topic1: %s", msg.data)
        self.merge_and_publish()


    def callback_rl(self, msg):
        self.data_rl = msg.data
        rospy.loginfo("Received from topic2: %s", msg.data)
        self.merge_and_publish()


    def merge_and_publish(self):
        if self.data_mpc is not None and self.data_rl is not None:
            merged_data = self.data_mpc[0:9] + tuple([0]*(7)) + self.data_mpc[9:18] + self.data_rl + tuple([0]*(5)) # Merge arrays by concatenation
            output_msg = UInt16MultiArray()
            output_msg.data = merged_data

            self.pub.publish(output_msg)
            rospy.loginfo("Published merged array: %s", output_msg.data)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    node = OutputIntegrator()
    node.run()
