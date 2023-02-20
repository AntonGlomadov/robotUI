import paramiko 
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

# connect via ssh to raspberry pi and show all files in /home/pi/Downloads
class FileWindow(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(FileWindow, self).__init__(parent)
        self.setWindowTitle("Files")
        self.setGeometry(0, 0, 800, 600)
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)
        self.files = QtWidgets.QListWidget()
        self.files.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.layout.addWidget(self.files)
        self.button = QtWidgets.QPushButton("Delete")
        self.button.clicked.connect(self.delete_files)
        self.layout.addWidget(self.button)
        self.show()

    def delete_files(self):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname=host, username=user, password=secret, port=port)
        for file in self.files.selectedItems():
            ssh.exec_command("rm /home/pi/Downloads/"+file.text())
        ssh.close()
        self.files.clear()
        self.show_files()

    def show_files(self):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname=host, username=user, password=secret, port=port)
        stdin, stdout, stderr = ssh.exec_command("ls /home/pi/Downloads")
        for file in stdout.readlines():
            self.files.addItem(file.strip())
        ssh.close()
    
    def add_file(self, file):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname=host, username=user, password=secret, port=port)
        ssh.exec_command("cp /home/pi/Downloads/"+file+" /home/pi/Downloads/"+file+".bak")
        ssh.close()


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = FileWindow()
    window.show_files()
    sys.exit(app.exec_())


