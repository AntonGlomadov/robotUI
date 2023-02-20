import sys

import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from telerobot_interfaces.msg import Motor

class SpeedWindow(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(SpeedWindow, self).__init__(parent)
        self.setWindowTitle("Скорость колес")
        self.setGeometry(0, 0, 800, 600)
        self.layout = QtWidgets.QGridLayout()
        self.setLayout(self.layout)
        self.label1 = QtWidgets.QLabel("row")
        self.layout.addWidget(self.label1, 0, 0)
        self.label2 = QtWidgets.QLabel("pitch")
        self.layout.addWidget(self.label2, 0, 1)
        self.label3 = QtWidgets.QLabel("yaw")
        self.layout.addWidget(self.label3, 0, 2)
        self.increasebutton = QtWidgets.QPushButton
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000)

    def connect_ros_float(self, state):
        rclpy.init(args=None)
        self.node = Node('Qt_view_node')
        self.pub = self.node.create_publisher(
            Motor,
            'wheel_commands',
            10
        )

       
            
        

    def resizeEvent(self, event):
        font = QtGui.QFont()
        font.setPointSize((self.height()+self.width())// 35)
        self.label1.setFont(font)
        self.label2.setFont(font)
        self.label3.setFont(font)  

    def update(self): 
        self.label1.setText("row: " + str(np.random.randint(0, 100)))
        self.label2.setText("pitch: " + str(np.random.randint(0, 100)))
        self.label3.setText("yaw: " + str(np.random.randint(0, 100)))

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main = IMUWindow()
    main.show()
    sys.exit(app.exec_())