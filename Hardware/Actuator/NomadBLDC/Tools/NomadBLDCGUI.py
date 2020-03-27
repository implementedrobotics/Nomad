from PyQt5 import QtWidgets, QtCore, uic
import sys
import time
from NomadBLDC import NomadBLDC

Mode_Map = ['IDLE', 'ERROR', 'MEASURE R', 'MEASURE L', 'MEASURE DIR', 'CURRENT MODE', 'VOLTAGE MODE', 'TORQUE MODE', 'SPEED MODE']
class NomadBLDCGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super(NomadBLDCGUI, self).__init__()
        uic.loadUi("NomadBLDCGUI.ui", self) 
        self.InitWindow()
        

        self.nomad_dev = NomadBLDC()
        self.connectButton.clicked.connect(self.ConnectDevice)
        self.calibrateButton.clicked.connect(self.CalibrateDevice)
        self.restartButton.clicked.connect(self.RestartDevice)
        self.voltageControlButton.clicked.connect(self.StartVoltageControl)
        self.torqueControlButton.clicked.connect(self.StartTorqueControl)
        self.idleModeButton.clicked.connect(self.EnterIdleMode)

        self.voltageControlSlider.valueChanged.connect(self.SetVoltageSetPoint)
        self.k_p_spin.valueChanged.connect(self.SetTorqueSetPoint)
        self.k_d_spin.valueChanged.connect(self.SetTorqueSetPoint)
        self.pos_spin.valueChanged.connect(self.SetTorqueSetPoint)
        self.vel_spin.valueChanged.connect(self.SetTorqueSetPoint)
        self.torqueFF_spin.valueChanged.connect(self.SetTorqueSetPoint)

        # Measurements
        self.measurePhaseResistanceButton.clicked.connect(self.MeasureMotorResistance)

        self.connectInfoLabel.setText("Please plug in Nomad BLDC device and press Connect.")
        self.portInfoLabel.setText("")
        self.deviceInfoLabel.setText("")
        self.firmwareInfoLabel.setText("")
        self.uptimeLabel.setText("")
        self.restartButton.hide()

        # Start Timer
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.UpdateEvent)
        self.update_timer.start(1000)

        # Callbacks
        self.nomad_dev.commands.set_logger_cb(self.UpdateLog)


    def InitWindow(self):
        self.statusBar().showMessage('Not Connected')
        self.show()

    def ConnectDevice(self):
        if(self.nomad_dev.connected): # Disconnect
            self.nomad_dev.disconnect()
            self.statusBar().showMessage(f'Not Connected')
            self.connectInfoLabel.setText("Please plug in Nomad BLDC device and press Connect.")
            self.portInfoLabel.setText("")
            self.deviceInfoLabel.setText("")
            self.firmwareInfoLabel.setText("")
            self.uptimeLabel.setText("")
            self.restartButton.hide()
        
        elif(self.nomad_dev.connect()): # Connected
            self.statusBar().showMessage(f'NomadBLDC Device Connected: {self.nomad_dev.port}')
            # Update labels
            self.connectInfoLabel.setText("Nomad BLDC Connected")
            self.portInfoLabel.setText(f"Port: {self.nomad_dev.port}")
            self.deviceInfoLabel.setText(f"Device ID: {self.nomad_dev.device_info.device_id}")
            self.firmwareInfoLabel.setText(f"Firmware Version: v{self.nomad_dev.device_info.fw_major}.{self.nomad_dev.device_info.fw_minor}")    
            self.uptimeLabel.setText("Up Time: ")
            self.restartButton.show()
            self.connectButton.setText("Disconnect")
            self.nomad_dev.get_device_stats()
            self.logTerminalEdit.append("CONNECTED YO")

    # def CheckConnection(self):
    #     msgBox = QtWidgets.QMessageBox()
    #     msgBox.setIcon(QtWidgets.QMessageBox.Information)
    #     msgBox.setText("Message box pop up window")
    #     msgBox.setWindowTitle("QMessageBox Example")
    #     msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
    #     msgBox.exec()

    def MeasureMotorResistance(self):
        self.nomad_dev.measure_motor_resistance()

    def CalibrateDevice(self):
        self.nomad_dev.calibrate_motor()

    def RestartDevice(self):
        self.nomad_dev.restart_device() 
    
    def StartVoltageControl(self):
        self.nomad_dev.start_voltage_control()
    
    def StartTorqueControl(self):
        self.nomad_dev.start_torque_control()

    def EnterIdleMode(self):
        self.nomad_dev.enter_idle_mode()

    def SetVoltageSetPoint(self):
        max_voltage = 5.0
        v_q = max_voltage * self.voltageControlSlider.value()/1000.0
        self.nomad_dev.set_voltage_setpoint(0, v_q) 
    
    def SetTorqueSetPoint(self):
        self.nomad_dev.set_torque_setpoint(self.k_p_spin.value(), self.k_d_spin.value(), self.pos_spin.value(), self.vel_spin.value(), self.torqueFF_spin.value()) 

    def closeEvent(self, event):
        self.nomad_dev.disconnect()

    def UpdateEvent(self):
        if(self.nomad_dev.connected):
            stats = self.nomad_dev.get_device_stats()
            if(stats is not None): # Update Stats
                time_str = time.strftime("%Hh %Mm %Ss", time.gmtime(stats.uptime))
                self.uptimeLabel.setText(f"Up Time: " + time_str)
                self.busVoltageLabel.setText("V<sub>(bus)</sub>: {:0.2f}v".format(stats.voltage_bus))
                self.controllerStatusLabel.setText(f"Controller Status: <b>{Mode_Map[stats.control_status]}</b>")

    def UpdateLog(self, log_info):
        
        # Append to terminal
        self.logTerminalEdit.append(log_info)

        # Autoscroll?
        if(self.autoScrollTerminalCheck.isChecked()):
            self.logTerminalEdit.ensureCursorVisible()
            
if __name__ == '__main__':

    app = QtWidgets.QApplication([])
    window = NomadBLDCGUI()
    sys.exit(app.exec())




