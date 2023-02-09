from enum import Enum
import sys
import time
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt


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
        # vertical layout with two lables
        self.layout = QtWidgets.QVBoxLayout()

        # информация о скорости
        self.speedInfoLayout = QtWidgets.QFormLayout()
        # информация о текущей скорости
        self.qurentSpeed = 0
        self.qurentSpeedQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedQlineEdit.setReadOnly(True)
        self.qurentSpeedQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedQlineEdit.setText(str(self.qurentSpeed))
        self.speedInfoLayout.addRow(
            "Текущая скорость: ", self.qurentSpeedQlineEdit)
        # информация о скорости переднего правого колеса
        self.qurentSpeedFrontRight = 0
        self.qurentSpeedFrontRightQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedFrontRightQlineEdit.setReadOnly(True)
        self.qurentSpeedFrontRightQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedFrontRightQlineEdit.setText(
            str(self.qurentSpeedFrontRight))
        self.speedInfoLayout.addRow(
            "Скорость переднего правого колеса: ", self.qurentSpeedFrontRightQlineEdit)
        # информация о скорости переднего левого колеса
        self.qurentSpeedFrontLeft = 0
        self.qurentSpeedFrontLeftQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedFrontLeftQlineEdit.setReadOnly(True)
        self.qurentSpeedFrontLeftQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedFrontLeftQlineEdit.setText(
            str(self.qurentSpeedFrontLeft))
        self.speedInfoLayout.addRow(
            "Скорость переднего левого колеса: ", self.qurentSpeedFrontLeftQlineEdit)
        # информация о скорости заднего правого колеса
        self.qurentSpeedBackRight = 0
        self.qurentSpeedBackRightQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedBackRightQlineEdit.setReadOnly(True)
        self.qurentSpeedBackRightQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedBackRightQlineEdit.setText(
            str(self.qurentSpeedBackRight))
        self.speedInfoLayout.addRow(
            "Скорость заднего правого колеса: ", self.qurentSpeedBackRightQlineEdit)
        # информация о скорости заднего левого колеса
        self.qurentSpeedBackLeft = 0
        self.qurentSpeedBackLeftQlineEdit = QtWidgets.QLineEdit()
        self.qurentSpeedBackLeftQlineEdit.setReadOnly(True)
        self.qurentSpeedBackLeftQlineEdit.setAlignment(QtCore.Qt.AlignRight)
        self.qurentSpeedBackLeftQlineEdit.setText(
            str(self.qurentSpeedBackLeft))
        self.speedInfoLayout.addRow(
            "Скорость заднего левого колеса: ", self.qurentSpeedBackLeftQlineEdit)

        # информация о максимальной скорости
        self.maxSpeed = 10
        self.maxSpeedSpinBox = QtWidgets.QSpinBox()
        self.maxSpeedSpinBox.setRange(0, 20)
        self.maxSpeedSpinBox.setSingleStep(1)
        self.maxSpeedSpinBox.setValue(self.maxSpeed)
        self.maxSpeedSpinBox.valueChanged.connect(self.updatemaxSpeed)
        self.speedInfoLayout.addRow(
            "Максимальная скорость: ", self.maxSpeedSpinBox)

        # управление скоростью
        self.initSpeedButtons()
        self.layout.addLayout(self.speedControlLayout)
        self.i = 0
        self.setLayout(self.layout)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def initSpeedButtons(self):
        self.speedTimer = QtCore.QTimer()
        self.speedTimer.timeout.connect(self.updateSpeed)
        self.direction = Direction.Stop
        self.speedControlLayout = QtWidgets.QGridLayout()
        # вперед
        self.speedFrontButton = QtWidgets.QPushButton("Вперед")
        self.speedFrontButton.pressed.connect(self.speedFrontIncrease)
        self.speedFrontButton.released.connect(self.speedStop)
        self.speedFrontButton.setShortcut(Qt.Key_W)
        self.speedControlLayout.addWidget(self.speedFrontButton, 0, 1)
        # назад
        self.speedBackButton = QtWidgets.QPushButton("Назад")
        self.speedBackButton.pressed.connect(self.speedBackIncrease)
        self.speedBackButton.released.connect(self.speedStop)
        self.speedControlLayout.addWidget(self.speedBackButton, 1, 1)
        # влево
        self.speedLeftButton = QtWidgets.QPushButton("Влево")
        self.speedLeftButton.pressed.connect(self.speedLeftIncrease)
        self.speedLeftButton.released.connect(self.speedStop)
        self.speedControlLayout.addWidget(self.speedLeftButton, 1, 0)
        # вправо
        self.speedRightButton = QtWidgets.QPushButton("Вправо")
        self.speedRightButton.pressed.connect(self.speedRightIncrease)
        self.speedRightButton.released.connect(self.speedStop)
        self.speedControlLayout.addWidget(self.speedRightButton, 1, 2)
        self.layout.addLayout(self.speedInfoLayout)

    # Функция обновления скорости
    def speedStop(self):
        self.speedTimer.stop()
        while (self.qurentSpeed != 0 or self.qurentSpeedFrontRight != 0 or self.qurentSpeedFrontLeft != 0 or self.qurentSpeedBackRight != 0 or self.qurentSpeedBackLeft != 0):
            if (self.qurentSpeedFrontRight > 0):
                self.qurentSpeedFrontRight = self.qurentSpeedFrontRight - 1
            if (self.qurentSpeedFrontLeft > 0):
                self.qurentSpeedFrontLeft = self.qurentSpeedFrontLeft - 1
            if (self.qurentSpeedBackRight > 0):
                self.qurentSpeedBackRight = self.qurentSpeedBackRight - 1
            if (self.qurentSpeedBackLeft > 0):
                self.qurentSpeedBackLeft = self.qurentSpeedBackLeft - 1
            if (self.qurentSpeedFrontRight < 0):
                self.qurentSpeedFrontRight += 1
            if (self.qurentSpeedFrontLeft < 0):
                self.qurentSpeedFrontLeft += 1
            if (self.qurentSpeedBackRight < 0):
                self.qurentSpeedBackRight += 1
            if (self.qurentSpeedBackLeft < 0):
                self.qurentSpeedBackLeft += 1
            self.qurentSpeed = (self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft +
                                self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4

    def speedFrontIncrease(self):
        self.direction = Direction.Front
        self.speedTimer.start(50)

    def speedBackIncrease(self):
        self.direction = Direction.Back
        self.speedTimer.start(100)

    def speedLeftIncrease(self):
        self.direction = Direction.Left
        self.speedTimer.start(100)

    def speedRightIncrease(self):
        self.direction = Direction.Right
        self.speedTimer.start(100)

    def updateSpeed(self):
        if self.direction == Direction.Front:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft += 1
                self.qurentSpeedFrontRight += 1
                self.qurentSpeedBackLeft += 1
                self.qurentSpeedBackRight += 1
                self.qurentSpeed = (self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft +
                                    self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4
        if self.direction == Direction.Back:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft -= 1
                self.qurentSpeedFrontRight -= 1
                self.qurentSpeedBackLeft -= 1
                self.qurentSpeedBackRight -= 1
                self.qurentSpeed = abs(self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft +
                                       self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4
        if self.direction == Direction.Left:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft -= 1
                self.qurentSpeedFrontRight += 1
                self.qurentSpeedBackLeft += 1
                self.qurentSpeedBackRight -= 1
                self.qurentSpeed = abs(self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft +
                                       self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4
        if self.direction == Direction.Right:
            if (self.qurentSpeed < self.maxSpeed):
                self.qurentSpeedFrontLeft += 1
                self.qurentSpeedFrontRight -= 1
                self.qurentSpeedBackLeft -= 1
                self.qurentSpeedBackRight += 1
                self.qurentSpeed = abs(self.qurentSpeedFrontRight + self.qurentSpeedFrontLeft +
                                       self.qurentSpeedBackRight + self.qurentSpeedBackLeft) / 4

    def updatemaxSpeed(self):
        self.maxSpeed = self.maxSpeedSpinBox.value()

    def update(self):
        self.qurentSpeedQlineEdit.setText(str(self.qurentSpeed))
        self.qurentSpeedFrontRightQlineEdit.setText(
            str(self.qurentSpeedFrontRight))
        self.qurentSpeedFrontLeftQlineEdit.setText(
            str(self.qurentSpeedFrontLeft))
        self.qurentSpeedBackRightQlineEdit.setText(
            str(self.qurentSpeedBackRight))
        self.qurentSpeedBackLeftQlineEdit.setText(
            str(self.qurentSpeedBackLeft))


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main = SpeedWindow()
    main.show()
    sys.exit(app.exec_())
