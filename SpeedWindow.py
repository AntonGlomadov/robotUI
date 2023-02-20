from enum import Enum
import sys
import time
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt


import rclpy
from rclpy.node import Node
from telerobot_interfaces.msg import Motor


class Direction(Enum):
    Front = 0
    Back = 1
    Left = 2
    Right = 3
    Stop = 4

class SpeedWindow(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(SpeedWindow, self).__init__(parent)
        self.setWindowTitle("Скорость колес")
        self.setGeometry(0, 0, 800, 600)
        #vertical layout with two lables
        self.layout = QtWidgets.QVBoxLayout()

        #информация о скорости
        self.speedInfoLayout = QtWidgets.QFormLayout()
        #информация о текущей скорости
        self.qurentSpeed = 0;
        self.qurentSpeedQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedQlineEdit.setReadOnly(True)
        self.qurentSpeedQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedQlineEdit.setText(str(self.qurentSpeed))
        self.speedInfoLayout.addRow("Текущая скорость: ", self.qurentSpeedQlineEdit)
        #информация о скорости переднего правого колеса
        self.qurentSpeedFrontRight = 0;
        self.qurentSpeedFrontRightQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedFrontRightQlineEdit.setReadOnly(True)
        self.qurentSpeedFrontRightQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedFrontRightQlineEdit.setText(str(self.qurentSpeedFrontRight))
        self.speedInfoLayout.addRow("Скорость переднего правого колеса: ", self.qurentSpeedFrontRightQlineEdit)
        #информация о скорости переднего левого колеса
        self.qurentSpeedFrontLeft = 0;
        self.qurentSpeedFrontLeftQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedFrontLeftQlineEdit.setReadOnly(True)
        self.qurentSpeedFrontLeftQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedFrontLeftQlineEdit.setText(str(self.qurentSpeedFrontLeft))
        self.speedInfoLayout.addRow("Скорость переднего левого колеса: ", self.qurentSpeedFrontLeftQlineEdit)
        #информация о скорости заднего правого колеса
        self.qurentSpeedBackRight = 0;
        self.qurentSpeedBackRightQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedBackRightQlineEdit.setReadOnly(True)
        self.qurentSpeedBackRightQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedBackRightQlineEdit.setText(str(self.qurentSpeedBackRight))
        self.speedInfoLayout.addRow("Скорость заднего правого колеса: ", self.qurentSpeedBackRightQlineEdit)
        #информация о скорости заднего левого колеса
        self.qurentSpeedBackLeft = 0;
        self.qurentSpeedBackLeftQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedBackLeftQlineEdit.setReadOnly(True)
        self.qurentSpeedBackLeftQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedBackLeftQlineEdit.setText(str(self.qurentSpeedBackLeft))
        self.speedInfoLayout.addRow("Скорость заднего левого колеса: ", self.qurentSpeedBackLeftQlineEdit)

        #информация о максимальной скорости
        self.maxSpeed = 100;
        self.maxSpeedSpinBox = QtWidgets.QSpinBox()
        self.maxSpeedSpinBox.setRange(0, 20)
        self.maxSpeedSpinBox.setSingleStep(1)
        self.maxSpeedSpinBox.setValue(self.maxSpeed)
        self.maxSpeedSpinBox.valueChanged.connect(self.updatemaxSpeed)
        self.speedInfoLayout.addRow("Максимальная скорость: ", self.maxSpeedSpinBox)

        #ros2
        rclpy.init(args=None)
        self.node = Node('WheelCommandsGui')
        self.pub = self.node.create_publisher(
            Motor,
            'wheel_commands',
            10
        )
        

        #управление скоростью
        self.initSpeedButtons()
        self.layout.addLayout(self.speedControlLayout)
        self.i =0;
        self.setLayout(self.layout)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

            
    def initSpeedButtons(self):
        self.direction = Direction.Stop
        self.speedControlLayout = QtWidgets.QGridLayout()
        #вперед
        self.speedFrontButton = QtWidgets.QPushButton("Вперед")
        self.speedFrontButton.pressed.connect(self.speedFrontIncrease)
        self.speedFrontButton.setShortcut(Qt.Key_W)
        self.speedControlLayout.addWidget(self.speedFrontButton, 0, 1)
        #назад
        self.speedBackButton = QtWidgets.QPushButton("Назад")
        self.speedBackButton.pressed.connect(self.speedBackIncrease)
        self.speedBackButton.setShortcut(Qt.Key_S)
        self.speedControlLayout.addWidget(self.speedBackButton, 1, 1)
        #влево
        self.speedLeftButton = QtWidgets.QPushButton("Влево")
        self.speedLeftButton.pressed.connect(self.speedLeftIncrease)
        self.speedLeftButton.setShortcut(Qt.Key_A)
        self.speedControlLayout.addWidget(self.speedLeftButton, 1, 0)
        #вправо
        self.speedRightButton = QtWidgets.QPushButton("Вправо")
        self.speedRightButton.pressed.connect(self.speedRightIncrease)
        self.speedRightButton.setShortcut(Qt.Key_D)
        self.speedControlLayout.addWidget(self.speedRightButton, 1, 2)
        self.layout.addLayout(self.speedInfoLayout)

    def speedFrontIncrease(self):
        self.direction = Direction.Front
        self.updateSpeed()
    def speedBackIncrease(self):
        self.direction = Direction.Back
        self.updateSpeed()
    def speedLeftIncrease(self):
        self.direction = Direction.Left
        self.updateSpeed()
    def speedRightIncrease(self):
        self.direction = Direction.Right
        self.updateSpeed()
    def updateSpeed(self):
        if self.direction == Direction.Front:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft += 10
                self.qurentSpeedFrontRight += 10
                self.qurentSpeedBackLeft += 10
                self.qurentSpeedBackRight += 10
                self.qurentSpeed = (self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft + self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4
            
        if self.direction == Direction.Back:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft -= 10
                self.qurentSpeedFrontRight -= 10 
                self.qurentSpeedBackLeft -= 10
                self.qurentSpeedBackRight -= 10
                self.qurentSpeed = abs(self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft + self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4
        if self.direction == Direction.Left:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft -= 10
                self.qurentSpeedFrontRight += 10 
                self.qurentSpeedBackLeft += 10
                self.qurentSpeedBackRight -= 10
                self.qurentSpeed = abs(self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft + self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4
        if self.direction == Direction.Right:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft += 10
                self.qurentSpeedFrontRight -= 10 
                self.qurentSpeedBackLeft -= 10
                self.qurentSpeedBackRight += 10
                self.qurentSpeed = abs(self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft + self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4


    def updatemaxSpeed(self):
        self.maxSpeed = self.maxSpeedSpinBox.value()

    def update(self): 
        self.qurentSpeedQlineEdit.setText(str(self.qurentSpeed))
        self.qurentSpeedFrontRightQlineEdit.setText(str(self.qurentSpeedFrontRight))
        self.qurentSpeedFrontLeftQlineEdit.setText(str(self.qurentSpeedFrontLeft))
        self.qurentSpeedBackRightQlineEdit.setText(str(self.qurentSpeedBackRight))
        self.qurentSpeedBackLeftQlineEdit.setText(str(self.qurentSpeedBackLeft))
        msg = Motor()
        msg.motor_lf = self.qurentSpeedFrontLeft
        msg.motor_lr = self.qurentSpeedBackLeft
        msg.motor_rf = self.qurentSpeedFrontRight
        msg.motor_rr = self.qurentSpeedBackRight
        self.pub.publish(msg)




if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main = SpeedWindow()
    main.show()
    sys.exit(app.exec_())

