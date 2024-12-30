#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import plotext as plt

class PressurePlotNode:
    def __init__(self):
        rospy.init_node('pressure_plot_node', anonymous=True)
        rospy.Subscriber('/ref_values', Float32MultiArray, self.ref_callback)
        rospy.Subscriber('/sen_values', Float32MultiArray, self.sen_callback)
        self.ref_values = np.zeros(8)
        self.sen_values = np.zeros(8)
        plt.ylim(0, 100)

    def ref_callback(self, msg):
        if len(msg.data) >= 8:
            self.ref_values = np.array(msg.data[:8])

    def sen_callback(self, msg):
        if len(msg.data) >= 8:
            self.sen_values = np.array(msg.data[:8])

    def update_plot(self):
        while not rospy.is_shutdown():
            plt.clf()
            # Plot reference data with markers
            plt.scatter(range(len(self.ref_values)), self.ref_values, label='Reference', color='blue')
            # Plot sensor data with markers
            plt.scatter(range(len(self.sen_values)), self.sen_values, label='Sensor', color='red')
            plt.title('Real-time Pressure Plot - Reference (blue) vs Sensor (red)')
            plt.show()
            rospy.sleep(0.1)  # 100ms update interval

if __name__ == '__main__':
    try:
        node = PressurePlotNode()
        node.update_plot()
    except rospy.ROSInterruptException:
        pass
