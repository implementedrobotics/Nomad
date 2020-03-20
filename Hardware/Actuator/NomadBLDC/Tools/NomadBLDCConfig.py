
from PyQt5.QtWidgets import QMainWindow, QApplication
import sys
from NomadBLDC import NomadBLDC

class Window(QMainWindow):
    def __init__(self):
        super().__init__()

        self.title = "Nomad BLDC Motor Configurator"
        self.top = 100
        self.left = 100
        self.width = 400
        self.height = 300
        
        self.InitWindow()

    def InitWindow(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.show()


if __name__ == '__main__':
    nomad = NomadBLDC(timeout=0.5, baud=921600)
    nomad.connect()

    #App = QApplication(sys.argv)
    #window = Window()

    #sys.exit(App.exec())


