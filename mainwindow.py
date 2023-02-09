from PyQt5 import QtCore, QtWidgets, QtGui
import numpy as np

from ImuWindow import IMUWindow
from SpeedWindow import SpeedWindow


#main window with mdi area and menu bar
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Main Window")
        self.setGeometry(0, 0, 800, 600)
        self.mdi = QtWidgets.QMdiArea()
        self.setCentralWidget(self.mdi)
        self.menu = self.menuBar()
        self.file = self.menu.addMenu("Окна")

        self.new_speed = QtWidgets.QAction("Скорость движения", self)
        self.new_speed.triggered.connect(self.new_speed_window)
        self.file.addAction(self.new_speed)
        self.mdi.setViewMode(QtWidgets.QMdiArea.SubWindowView)
        #add new to file menu
        self.new_imu = QtWidgets.QAction("IMU", self)
        self.new_imu.triggered.connect(self.new_imu_window)
        self.file.addAction(self.new_imu)
        self.mdi.setViewMode(QtWidgets.QMdiArea.SubWindowView)

        self.mdi.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.mdi.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)


    def new_imu_window(self):
        sub = IMUWindow()
        self.mdi.addSubWindow(sub)
        sub.show()

    def new_speed_window(self):
        sub = SpeedWindow()
        self.mdi.addSubWindow(sub)
        sub.show()


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
