import os
import sys
from PyQt5.QtCore import (QTimer)
from PyQt5.QtWidgets import (QApplication, QPushButton)
from PyQt5.QtCore import Qt


'''окно отображающее переменную i. Когда пользователь нажимает и удерживает кнопку w на клавиатуре, i увеличивается на 1 каждые 100 мс. 
Когда пользователь отпускает кнопку, i уменьшается на 1 каждые 100 мс.'''


class MainWindow(QPushButton):
    def __init__(self):
        super().__init__()
        self.i = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            self.i += 1

    def update(self):
        self.i -= 1
        self.setText(str(self.i))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
