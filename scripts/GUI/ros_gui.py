#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget

class ROSGraphPlotter(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('ROS Graph Plotter')
        
        # Matplotlib figure and axes
        self.fig, self.axs = plt.subplots(4, 2, figsize=(10, 10))
        self.canvas = FigureCanvas(self.fig)
        
        # Initialize ROS node
        rospy.init_node('graph_plotter', anonymous=True)

        # ROS topic subscriptions
        rospy.Subscriber('/sen_values', Float32MultiArray, self.sen_values_callback)
        rospy.Subscriber('/ref_values', Float32MultiArray, self.ref_values_callback)
        
        self.sen_values = []
        self.ref_values = []
        
        # Set up layout
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def sen_values_callback(self, msg):
        # Assuming sen_values contains 9 values
        self.sen_values = msg.data
        self.update_plot()

    def ref_values_callback(self, msg):
        # Assuming ref_values contains 6 values, add 2 random values to make it 8
        self.ref_values = msg.data + [0.0, 0.0]  # Adding two dummy values
        self.update_plot()

    def update_plot(self):
        if len(self.sen_values) == 9 and len(self.ref_values) == 8:
            # Plotting each graph
            for i in range(4):
                for j in range(2):
                    self.axs[i, j].cla()  # Clear previous plot
                    if j == 0:  # Plot sen_values on the left
                        self.axs[i, j].plot(self.sen_values, label='sen_values')
                        self.axs[i, j].set_title(f'sen_values (Graph {i*2 + j + 1})')
                    else:  # Plot ref_values on the right
                        self.axs[i, j].plot(self.ref_values, label='ref_values', color='r')
                        self.axs[i, j].set_title(f'ref_values (Graph {i*2 + j + 1})')
                    
                    self.axs[i, j].legend()
            
            self.canvas.draw()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    app = QApplication([])
    window = ROSGraphPlotter()
    window.show()
    window.run()
