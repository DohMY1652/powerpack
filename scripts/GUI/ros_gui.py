#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()

        # ROS 초기화
        rospy.init_node('gui_node', anonymous=True)

        # GUI 설정
        self.label = QLabel("Waiting for messages...", self)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Subscriber 설정
        self.subscriber = rospy.Subscriber('sen_values', Float32MultiArray, self.callback)

        # Timer 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ros)
        self.timer.start(100)  # 100ms마다 호출

        self.last_data = None  # 마지막 수신된 데이터를 저장

    def callback(self, data):
        # 수신한 데이터를 저장
        self.last_data = data

    def update_ros(self):
        # 수신한 데이터가 있을 경우 GUI 업데이트
        if self.last_data is not None:
            values = ', '.join(map(str, self.last_data.data))
            self.label.setText("Received: [{}]".format(values))

def main():
    app = QApplication(sys.argv)
    gui = ROSGui()
    gui.setWindowTitle("ROS Subscriber GUI")
    gui.resize(300, 200)
    gui.show()

    # PyQt 이벤트 루프 실행
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
