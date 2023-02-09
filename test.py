import os
import sys
from PyQt5.QtCore import(QTimer)
from PyQt5.QtWidgets import(QApplication, QPushButton)

def button_pressed(timer):
    timer.start(100)

def button_released(timer):
    timer.stop()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    pb = QPushButton("Press")
    timer = QTimer()
    pb.pressed.connect(lambda checked = False: button_pressed(timer))
    pb.released.connect(lambda checked = False: button_released(timer))
    timer.timeout.connect(lambda: print('Button Pressed'))
    pb.show()
    app.exec_()