from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import pyqtSlot, pyqtSignal

import sys
import time
import threading

from NomadBLDC import NomadBLDC

close_event = threading.Event()

# Background measurement signals
class MeasurementUpdateSignals(QtCore.QObject):
    completed = pyqtSignal(object)

# Background measurement worker class
class MeasurementUpdater(QtCore.QRunnable, QtCore.QObject):
    
    def __init__(self, nomad_device):
        super(MeasurementUpdater, self).__init__()
        self.nomad_dev = nomad_device
        self.signals = MeasurementUpdateSignals()
        self.measure_fn = None
        #self.timeout = 1

    @pyqtSlot()
    def run(self):
        if(self.nomad_dev.connected):
            result = self.measure_fn()
            self.signals.completed.emit(result)
            print(f"Result: {result}")

# Background work signals
class BackgroundUpdaterSignals(QtCore.QObject):
    updated = pyqtSignal(object)

# Background worker class
class BackgroundUpdater(QtCore.QRunnable, QtCore.QObject):
    
    def __init__(self, nomad_device):
        super(BackgroundUpdater, self).__init__()
        self.nomad_dev = nomad_device
        self.signals = BackgroundUpdaterSignals()
        self.period = 1
    @pyqtSlot()
    def run(self):
        while(1):
            if(self.nomad_dev.connected):
                stats = self.nomad_dev.get_device_stats()
                if(stats is not None):
                    self.signals.updated.emit(stats)
                time.sleep(self.period)
            else:
                time.sleep(self.period)
            
            if(close_event.is_set()):
                break
        
# Progress Dialog
class NomadBasicProgressDialog(QtWidgets.QDialog):
    def __init__(self, parentWindow=None):
        super(NomadBasicProgressDialog, self).__init__()
        uic.loadUi("NomadBasicProgress.ui", self) 
        if(parentWindow is not None): # Center on Parent
            self.move(parentWindow.window().frameGeometry().topLeft() + parentWindow.window().rect().center() - self.rect().center())
    
   # def CloseWindow(self):

Mode_Map = ['IDLE', 'ERROR', 'MEASURE R', 'MEASURE L', 'MEASURE PHASE DIRECTION', 'CALIBRATION MODE', 'CURRENT MODE', 'VOLTAGE MODE', 'TORQUE MODE', 'SPEED MODE']
class NomadBLDCGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super(NomadBLDCGUI, self).__init__()
        uic.loadUi("NomadBLDCGUI.ui", self) 
        self.InitWindow()
        
        # Thread Pool
        self.threadpool = QtCore.QThreadPool()

        # Nomad BLDC Device
        self.nomad_dev = NomadBLDC()

        # Signals/Slots
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
        self.measurePhaseInductanceButton.clicked.connect(self.MeasureMotorInductance)
        self.measurePhaseOrderButton.clicked.connect(self.MeasurePhaseOrder)
        self.loadConfigButton.clicked.connect(self.LoadConfiguration)

        self.connectInfoLabel.setText("Please plug in Nomad BLDC device and press Connect.")
        self.portInfoLabel.setText("")
        self.deviceInfoLabel.setText("")
        self.firmwareInfoLabel.setText("")
        self.uptimeLabel.setText("")
        self.restartButton.hide()

        # Clear Status
        self.gateDriverTempLabel.setText("")
        self.fetTempLabel.setText("")
        self.motorTempLabel.setText("")
        self.busVoltageLabel.setText("")
        self.controllerFaultLabel.setText("")
        self.resetFaultButton.hide()
        
        # Start Updater
        self.updater = BackgroundUpdater(self.nomad_dev)
        self.updater.period = 1 # Seconds
        self.updater.signals.updated.connect(self.UpdateSlot)
        self.threadpool.start(self.updater)

        # Callbacks
        self.nomad_dev.commands.set_logger_cb(self.UpdateLog)

        # Default Splitter
        self.mainSplitter.moveSplitter(700, 1)

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
            self.connectButton.setText("Connect")
            self.restartButton.hide()
        
        elif(self.nomad_dev.connect()): # Connected
            self.statusBar().showMessage(f'NomadBLDC Device Connected: {self.nomad_dev.port}')
            # Update labels
            self.connectInfoLabel.setText("Nomad BLDC Connected")
            self.portInfoLabel.setText(f"Port: {self.nomad_dev.port}")
            self.deviceInfoLabel.setText(f"ID: {self.nomad_dev.device_info.device_id}")
            self.firmwareInfoLabel.setText(f"Firmware Version: v{self.nomad_dev.device_info.fw_major}.{self.nomad_dev.device_info.fw_minor}")    
            self.uptimeLabel.setText("Up Time: ")
            self.restartButton.show()
            self.connectButton.setText("Disconnect")
            self.nomad_dev.get_device_stats()
            self.resetFaultButton.show()
        else: # Did not connect
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Information)
            msgBox.setText("Unable to connect to a valid Nomad BLDC Device.\nPlease verify connection and try again.")
            msgBox.setWindowTitle("Nomad BLDC")
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec()

    def MeasureMotorResistance(self):
        measurementTask = MeasurementUpdater(self.nomad_dev)
        measurementTask.measure_fn = self.nomad_dev.measure_motor_resistance
        #updater.period = 1 # Seconds
        #updater.signals.updated.connect(self.UpdateSlot)
        self.threadpool.start(measurementTask)
        measure_progress = NomadBasicProgressDialog(self)
        measure_progress.setWindowTitle("Resistance Measurement")
        measure_progress.progressText.setText("Measure Resistance...")
        measurementTask.signals.completed.connect(measure_progress.close)
        measurementTask.signals.completed.connect(self.UpdateResistanceMeasurementValue)
        measure_progress.exec_()    

    def MeasureMotorInductance(self):
        measurementTask = MeasurementUpdater(self.nomad_dev)
        measurementTask.measure_fn = self.nomad_dev.measure_motor_inductance
        self.threadpool.start(measurementTask)
        measure_progress = NomadBasicProgressDialog(self)
        measure_progress.setWindowTitle("Inductance Measurement")
        measure_progress.progressText.setText("Measure Inductance...")
        measurementTask.signals.completed.connect(measure_progress.close)
        measurementTask.signals.completed.connect(self.UpdateInductanceMeasurementValue)
        measure_progress.exec_()     

    def MeasurePhaseOrder(self):
        measurementTask = MeasurementUpdater(self.nomad_dev)
        measurementTask.measure_fn = self.nomad_dev.measure_motor_phase_order
        self.threadpool.start(measurementTask)
        measure_progress = NomadBasicProgressDialog(self)
        measure_progress.setWindowTitle("Phase Order Measurement")
        measure_progress.progressText.setText("Measure Phase Order...")
        measurementTask.signals.completed.connect(measure_progress.close)
        measurementTask.signals.completed.connect(self.UpdatePhaseOrderMeasurementValue)
        measure_progress.exec_()  

    def UpdateResistanceMeasurementValue(self, measurement):
        if(measurement is not None):
            self.phaseResistanceVal.setValue(measurement.measurement)
    
    def UpdateInductanceMeasurementValue(self, measurement):
        if(measurement is not None):
            self.phaseInductanceVal.setValue(measurement.measurement)

    def UpdatePhaseOrderMeasurementValue(self, measurement):
        if(measurement is not None):
            self.phaseOrderCombo.setCurrentIndex(measurement.measurement)

    def LoadConfiguration(self):
        if(self.nomad_dev.load_configuration() is not None):
            self.polePairVal.setValue(self.nomad_dev.motor_config.num_pole_pairs)
            self.KvVal.setValue(self.nomad_dev.motor_config.K_v)
            self.gearRatioVal.setValue(self.nomad_dev.motor_config.gear_ratio)
            self.KtMotorVal.setValue(self.nomad_dev.motor_config.K_t)
            self.KtOutputVal.setValue(self.nomad_dev.motor_config.K_t_out)
            self.phaseResistanceVal.setValue(self.nomad_dev.motor_config.phase_resistance)
            self.phaseInductanceVal.setValue(self.nomad_dev.motor_config.phase_inductance_d)
            self.phaseOrderCombo.setCurrentIndex(self.nomad_dev.motor_config.phase_order)

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
        close_event.set()

    #@pyqtSlot()
    def UpdateSlot(self, stats):
        if(stats is not None): # Update Stats
            time_str = time.strftime("%Hh %Mm %Ss", time.gmtime(stats.uptime))
            self.uptimeLabel.setText(f"Up Time: " + time_str)
            self.busVoltageLabel.setText("V<sub>(bus)</sub>: <b>{:0.2f}v</b>".format(stats.voltage_bus))
            self.controllerStatusLabel.setText(f"Controller Status: <b>{Mode_Map[stats.control_status]}</b>")
            self.gateDriverTempLabel.setText(f"Gate Driver Temp: <b>101.2 C</b>")
            self.fetTempLabel.setText("FET Temp: <b>28.3 C</b>")
            self.motorTempLabel.setText("Motor Temp: <b>29.5 C</b>")
            self.controllerFaultLabel.setText("Fault: <b>None</b>")
        

    def UpdateLog(self, log_info):
        
        # Append to terminal
        self.logTerminalEdit.append(log_info)

        # Autoscroll?
        if(self.autoScrollTerminalCheck.isChecked()):
            self.logTerminalEdit.ensureCursorVisible()

if __name__ == '__main__':

    app = QtWidgets.QApplication([])
    window = NomadBLDCGUI()
    sys.exit(app.exec_())




