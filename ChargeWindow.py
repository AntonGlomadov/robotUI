import sys
import time
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QWidget, QPushButton, QProgressBar, QVBoxLayout, QApplication
import psutil


#progress bar  that show battery level in runtime. color of progress bar changes depending on battery level, red if battery level is less than 20%, orange if battery level is less than 50%, green otherwise
class BatteryLevel(QProgressBar):
    def __init__(self):
        super().__init__()
        self.setMaximum(100)
        self.setMinimum(0)
        self.setValue(100)
        self.show()

    def update(self):
        battery = psutil.sensors_battery()
        self.setValue(battery.percent)
        #if bettery charing show charging icon
        if battery.power_plugged:
            self.setFormat("Charging:"+str(battery.percent)+"%")
        else:
            self.setFormat(str(battery.percent)+"%")
        if battery.percent < 20:
            self.setStyleSheet("QProgressBar::chunk {background-color: #FF0000; width: 1px;}")
        elif battery.percent < 50:
            self.setStyleSheet("QProgressBar::chunk {background-color: #FFA500; width: 1px;}")
        else:
            self.setStyleSheet("QProgressBar::chunk {background-color: #00FF00; width: 1px;}")
        self.show()



#thread that updates battery level every 5 seconds
class BatteryThread(QThread):
    update = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

    def run(self):
        while True:
            self.update.emit()
            time.sleep(5)


#main window showing battery level and label with percentage left on battery
class ChargeWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Battery Level")
        self.battery = BatteryLevel()
        self.battery_thread = BatteryThread()
        self.battery_thread.update.connect(self.battery.update)
        self.battery_thread.start()
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.battery)
        self.setLayout(self.layout)
        self.show()
        self.setStyleSheet("""

            QProgressBar {
                border-style: solid;
                border-color: grey;
                border-radius: 7px;
                border-width: 2px;
                text-align: center;
            }

            QProgressBar::chunk {
                width: 100px;
                background-color: #de7c09;
                margin: 1px;
            }

        """)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ChargeWindow()
    sys.exit(app.exec_())

