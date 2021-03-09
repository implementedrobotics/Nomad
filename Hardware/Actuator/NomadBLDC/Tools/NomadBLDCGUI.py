from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import pyqtSlot, pyqtSignal

import pyqtgraph as pg
import sys
import time
import threading
import math
import operator

import numpy as np

from NomadBLDC import NomadBLDC

close_event = threading.Event()

def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False

def getattr_nested(obj, nested_attr):
    attrs = nested_attr.split(".")
    ret = obj
    for attr in attrs:
            ret = getattr(ret, attr, None)
            if ret is None:
                return None
    return ret

class PlotData:
    def __init__(self, size=500,default=0):
        self.size = size
        self.x = []
        self.y = np.ones(size) * default
        self.curve = None

# Real Time Plotter
class RealTimeDataPlotter:
    def __init__(self, graphics_view):

        self.max_data_pts = 500
        self.enabled = True
        self.plot_list = []
        self.plot_data = [None] * self.max_data_pts
        self.plot_items = {}

        # Graphics View
        self.layout = pg.GraphicsLayout(border=(100,100,100))
        self.graphicsView = graphics_view
        self.graphicsView.setCentralItem(self.layout)
        self.graphicsView.show()
        self.graphicsView.setAntialiasing(True)
        self.graphicsView.setBackground('w')
        pg.setConfigOption('antialias', True)
        #self.graphicsView.enableMouse(False)

        #self.dir = 1
        self.duty = .5
        self.data1 = np.zeros(10)
        self.data2 = np.ones(10)
        self.duty_width = int(self.duty * 500)

        #print(250-self.duty_width // 2)
        #print(self.duty_width)
        duty_l = 250-self.duty_width // 2
        self.data1[duty_l:duty_l + self.duty_width] = 1
        #self.data1 = np.random.normal(size=500)
       # self.curve2 = self.test.plot(self.data1, pen='r')
       # self.curve1 = self.test.plot(self.data2, pen='b')
       # self.test.clear()
        #self.sample_ptr1 = 0

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.UpdatePlots)
        self.timer.start(50)

        #test.getAxis('left').setPen('b')
        
        #test.getAxis('bottom').setPen('b')
        #self.graphicsView.resize(800,600)

    # TODO: Should be by index
    def AddData(self, data, title='Title', row=0, col=0, pen='k', units=None, name=None, legend = False):
        
        plot_item = None
        if(self.plot_items.get(f"{row}:{col}") is None):
            # Not here.  Add it
            plot_item = self.layout.addPlot(row=row, col=col, title=title)
            plot_item.showGrid(x=True, y=True)
            #plot_item.setMouseEnabled(False)
            plot_item.setMenuEnabled(False)
            plot_item.setLabel('left', '', units=None)
            if(legend == True):
                plot_item.addLegend()
            # Update Dict
            self.plot_items[f"{row}:{col}"] = plot_item
        else:
            plot_item = self.plot_items[f"{row}:{col}"]

        
        index = len(self.plot_list)
        self.plot_data[index] = PlotData(size=500, default=0.0)
        self.plot_data[index].curve = plot_item.plot(self.plot_data[index].y, pen=pen, name=name)
        self.plot_list.append(data)

    def UpdatePlots(self):

        for idx, plot_item in enumerate(self.plot_list):
            #print(getattr_nested(item[0], item[1]))
            data_point = getattr_nested(plot_item[0], plot_item[1])
            if(data_point is None or math.isnan(data_point)):
                data_point = self.plot_data[idx].y[-1] # Copy in last data point, basically hold
                continue
                
            self.plot_data[idx].y = np.roll(self.plot_data[idx].y, -1)
            self.plot_data[idx].y[-1] = data_point
            self.plot_data[idx].curve.setData(self.plot_data[idx].y)
            #print(self.plot_data[idx].y)

    def Clear(self):
        self.plot_list.clear()
        self.plot_locations.clear()

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
            #print(f"Result: {result}")

# Background work signals
class BackgroundUpdaterSignals(QtCore.QObject):
    updated = pyqtSignal(object)
    updated_state = pyqtSignal(object)

# Background worker class
class BackgroundUpdater(QtCore.QRunnable, QtCore.QObject):
    
    def __init__(self, nomad_device):
        super(BackgroundUpdater, self).__init__()
        self.nomad_dev = nomad_device
        self.signals = BackgroundUpdaterSignals()
        self.period = 0.5
        self.paused = False
    @pyqtSlot()
    def run(self):
        while(1):
            
            #if(self.paused is True):
             #   time.sleep(1)
              #  continue

            if(self.nomad_dev.connected):
                stats = self.nomad_dev.get_device_stats()
                if(stats is not None):
                    self.signals.updated.emit(stats)

                state = self.nomad_dev.read_state()
                if(state is not None):
                    self.signals.updated_state.emit(state)

                time.sleep(self.period)
            else:
                time.sleep(self.period)
            
            if(close_event.is_set()):
                break

import serial

# Torque Calibrater Class
class TorqueCalibrater(QtCore.QRunnable, QtCore.QObject):
    
    def __init__(self, nomad_device, arm_length=0.5, num_steps=24, start_current=1.0, end_current=25):
        super(TorqueCalibrater, self).__init__()
        self.nomad_dev = nomad_device
        self.serial = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=1)
        self.arm_length = arm_length
        self.num_steps = num_steps
        self.start_current = start_current
        self.end_current = end_current
        self.torques = []
        self.currents = []
        #self.signals = BackgroundUpdaterSignals()
        #self.period = 0.5
        #self.paused = False

    @pyqtSlot()
    def run(self):

        self.nomad_dev.start_current_control()
        time.sleep(2)
        print("In Current Control")
        for i in range(self.num_steps+1):
            step_current = self.start_current + i/self.num_steps * (self.end_current-self.start_current)

            # Settle for 2 seconds
            time.sleep(2)
            print(step_current)

            self.nomad_dev.set_current_setpoint(0.0, step_current*-1) 
            time.sleep(1)
            self.nomad_dev.set_current_setpoint(0.0, step_current*-1) 
            time.sleep(1)
            test = self.read_scale(5)
            self.currents.append(step_current)
            self.torques.append(self.arm_length * test)

            print(f"Got: {test}")
        print(self.currents)
        print(self.torques)
        #print(self.torques/self.currents)
        self.nomad_dev.enter_idle_mode()
        x_arr = np.array(self.currents)
        y_arr = np.array(self.torques)
        Kt = y_arr/x_arr
        print(Kt)
        print(np.mean(Kt))

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOption('antialias', True)
        
        pw = pg.plot(x=x_arr, y=y_arr, pen='k', symbolBrush=(255,0,0), symbolPen='k')#, symbol='o')  # plot x vs y in red
        pw.showGrid(x=True, y=True)
        #pw.setMouseEnabled(False)
        pw.setMenuEnabled(False)

        pw.setLabel('left', 'Torque', 'N*m')
        pw.setLabel('bottom', 'Current', 'A')


    def read_scale(self, num_samples):
        i = 0
        total_sum = 0
        while i < num_samples:
            self.serial.write(b'r')
            data = self.serial.readline().decode()
            if(isfloat(data) == False):
                continue
            total_sum = total_sum + float(data)
            i = i + 1
        return total_sum / num_samples

# Progress Dialog
class NomadBasicProgressDialog(QtWidgets.QDialog):
    def __init__(self, parentWindow=None):
        super(NomadBasicProgressDialog, self).__init__()
        uic.loadUi("NomadBasicProgress.ui", self) 
        if(parentWindow is not None): # Center on Parent
            self.move(parentWindow.window().frameGeometry().topLeft() + parentWindow.window().rect().center() - self.rect().center())
    
   # def CloseWindow(self):

Mode_Map = ['STARTUP', 'IDLE', 'ERROR', 'MEASURE R', 'MEASURE L', 'MEASURE PHASE DIRECTION', 'MEASURE ENCODER OFFSET', 'CALIBRATION MODE', 'CURRENT MODE', 'VOLTAGE MODE', 'TORQUE MODE', 'SPEED MODE']
class NomadBLDCGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super(NomadBLDCGUI, self).__init__()
        uic.loadUi("NomadBLDCGUI.ui", self) 
        self.InitWindow()
        self.idle = True
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

        # Update Torques
        self.polePairVal.valueChanged.connect(self.UpdateTorqueOutput)
        self.KvVal.valueChanged.connect(self.UpdateTorqueOutput)
        self.gearRatioVal.valueChanged.connect(self.UpdateTorqueOutput)

        # Measurements
        self.measurePhaseResistanceButton.clicked.connect(self.MeasureMotorResistance)
        self.measurePhaseInductanceButton.clicked.connect(self.MeasureMotorInductance)
        self.measurePhaseOrderButton.clicked.connect(self.MeasurePhaseOrder)
        self.measureEncoderOffsetButton.clicked.connect(self.MeasureEncoderOffset)
        self.zeroMechanicalOffsetButton.clicked.connect(self.ZeroMechanicalOffset)
        self.encoderEccentricityMapButton.clicked.connect(self.ShowEncoderEccentricity)
        
        # Controller Settings
        self.autoComputeGainsButton.clicked.connect(self.AutoComputeControllerGains)

        #self.resetConfigButton.clicked.connect(self.ResetConfiguration)
        self.loadConfigButton.clicked.connect(lambda: self.LoadConfiguration(True))
        self.saveConfigButton.clicked.connect(self.SaveConfiguration)
        self.connectInfoLabel.setText("Please plug in Nomad BLDC device and press Connect.")
        self.portInfoLabel.setText("")
        self.deviceInfoLabel.setText("")
        self.firmwareInfoLabel.setText("")
        self.uptimeLabel.setText("")
        self.restartButton.hide()

        # Dev Settings
        self.loadTestStart.clicked.connect(self.DoLoadTest)
        self.thermalTestButton.clicked.connect(self.DoThermalTest)
        # Clear Status
        self.busCurrentLabel.setText("")
        self.fetTempLabel.setText("")
        self.motorTempLabel.setText("")
        self.busVoltageLabel.setText("")
        self.controllerFaultLabel.setText("")
        self.resetFaultButton.hide()
        
        # Start Plotter Interface
        self.rt_plotter = RealTimeDataPlotter(self.graphicsView)
        #self.rt_plotter.AddData((self.nomad_dev, "motor_state.I_a"), title='I(a)', row=0, col=0)
        #self.rt_plotter.AddData((self.nomad_dev, "motor_state.I_b"), title='I(b)', row=1, col=0)
        #self.rt_plotter.AddData((self.nomad_dev, "motor_state.I_c"), title='I(c)', row=2, col=0)

        self.rt_plotter.AddData((self.nomad_dev, "controller_state.Pos_ref"), title='Postion', name='Pos Ref', units='rads', legend=True, row=0, col=0, pen=pg.mkPen('r', width=2))
        self.rt_plotter.AddData((self.nomad_dev, "motor_state.theta_mech"), title='Position', name='Pos', units='rads', legend=True, row=0, col=0, pen=pg.mkPen('b', width=2))
        #self.rt_plotter.AddData((self.nomad_dev, "motor_state.theta_mech_true"), title='Position', name='Pos', units='rads', legend=True, row=0, col=0, pen=pg.mkPen('g', width=2))
        #self.rt_plotter.AddData((self.nomad_dev, "motor_state.theta_mech_dot"), title='Velocity', name='Vel', units='rads/s', legend=True, row=0, col=0, pen=pg.mkPen('r', width=2))
        self.rt_plotter.AddData((self.nomad_dev, "controller_config.pos_limit_min"), title='Position Limit (Min)', name='Position Limit (Min)', units='rads', legend=True, row=0, col=0, pen=pg.mkPen('b', width=2))
        self.rt_plotter.AddData((self.nomad_dev, "controller_config.pos_limit_max"), title='Position Limit (Max)', name='Position Limit (Max)', units='rads', legend=True, row=0, col=0, pen=pg.mkPen('b', width=2))
       # self.rt_plotter.AddData((self.nomad_dev, "controller_config.position_limit"), title='Position Limit', name='Position Limit', units='rads', legend=True, row=0, col=0, pen=pg.mkPen('b', width=2))
        self.rt_plotter.AddData((self.nomad_dev, "controller_state.I_q"), title='Q Axis Current', name='I(q)', units='A', legend=True, row=1, col=0, pen=pg.mkPen('b', width=2))
        self.rt_plotter.AddData((self.nomad_dev, "controller_state.I_q_ref"), title='Q Axis Current', name='I(q) ref', units='A', legend=True, row=1, col=0, pen=pg.mkPen('r', width=2))

        #self.rt_plotter.AddData((self.nomad_dev, "controller_state.I_max"), title='Max Current Demand', name='I(max)', units='A', legend=True, row=1, col=0, pen=pg.mkPen('g', width=2))
        #self.rt_plotter.AddData((self.nomad_dev, "controller_state.I_rms"), title='Motor RMS Current', name='I(rms)', units='A', legend=True, row=1, col=0, pen=pg.mkPen('r', width=2))
        #self.rt_plotter.AddData((self.nomad_dev, "controller_state.I_q"), title='Q Axis Current', name='I(q)', units='A', legend=True, row=1, col=0, pen=pg.mkPen('b', width=2))

        # Start Updater
        self.updater = BackgroundUpdater(self.nomad_dev)
        self.updater.period = 0.05 # Seconds
        self.updater.signals.updated.connect(self.UpdateSlot)
        self.updater.signals.updated_state.connect(self.UpdateState)
        
        self.threadpool.start(self.updater)

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

            # Load Configuration
            self.LoadConfiguration(False)
            #print(self.nomad_dev.device_info[0])

        else: # Did not connect
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Information)
            msgBox.setText("Unable to connect to a valid Nomad BLDC Device.\nPlease verify connection and try again.")
            msgBox.setWindowTitle("Nomad BLDC")
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec()

    def MeasureMotorResistance(self):
        self.updater.paused = True
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
        self.updater.paused = False 

    def MeasureMotorInductance(self):
        self.updater.paused = True
        measurementTask = MeasurementUpdater(self.nomad_dev)
        measurementTask.measure_fn = self.nomad_dev.measure_motor_inductance
        self.threadpool.start(measurementTask)
        measure_progress = NomadBasicProgressDialog(self)
        measure_progress.setWindowTitle("Inductance Measurement")
        measure_progress.progressText.setText("Measure Inductance...")
        measurementTask.signals.completed.connect(measure_progress.close)
        measurementTask.signals.completed.connect(self.UpdateInductanceMeasurementValue)
        measure_progress.exec_()     
        self.updater.paused = False 

    def MeasurePhaseOrder(self):
        self.updater.paused = True
        measurementTask = MeasurementUpdater(self.nomad_dev)
        measurementTask.measure_fn = self.nomad_dev.measure_motor_phase_order
        self.threadpool.start(measurementTask)
        measure_progress = NomadBasicProgressDialog(self)
        measure_progress.setWindowTitle("Phase Order Measurement")
        measure_progress.progressText.setText("Measure Phase Order...")
        measurementTask.signals.completed.connect(measure_progress.close)
        measurementTask.signals.completed.connect(self.UpdatePhaseOrderMeasurementValue)
        measure_progress.exec_()  
        self.updater.paused = False 

    def MeasureEncoderOffset(self):
        self.updater.paused = True
        measurementTask = MeasurementUpdater(self.nomad_dev)
        measurementTask.measure_fn = self.nomad_dev.measure_motor_encoder_offset
        self.threadpool.start(measurementTask)
        measure_progress = NomadBasicProgressDialog(self)
        measure_progress.setWindowTitle("Encoder Offset Measurement")
        measure_progress.progressText.setText("Measure Encoder Offset...")
        measurementTask.signals.completed.connect(measure_progress.close)
        measurementTask.signals.completed.connect(self.UpdateEncoderOffsetMeasurementValue)
        measure_progress.exec_()  
        self.updater.paused = False 

    def UpdateResistanceMeasurementValue(self, measurement):
        if(measurement is not None):
            self.phaseResistanceVal.setValue(measurement.measurement*1e3)
    
    def UpdateInductanceMeasurementValue(self, measurement):
        if(measurement is not None):
            self.phaseInductanceVal.setValue(measurement.measurement*1e6)

    def UpdatePhaseOrderMeasurementValue(self, measurement):
        if(measurement is not None):
            self.phaseOrderCombo.setCurrentIndex(measurement.measurement)

    def UpdateEncoderOffsetMeasurementValue(self, measurement):
        if(measurement is not None):
            self.electricalOffsetVal.setValue(measurement.measurement)

    def ZeroMechanicalOffset(self):
        #button = self.sender()
        #print(button.objectName())
        self.nomad_dev.zero_mechanical_offset()

        # Reload config to display it.  This is kind of crap.  Should get a return from zero_mech fucntion.
        # Will do for now
        self.LoadConfiguration(False)

    def UpdateTorqueOutput(self):

        # sqrt(3) = 1.73205080757
        flux_linkage = 60.0 / (1.73205080757 * self.KvVal.value() *  math.pi *  self.polePairVal.value() * 2)

        K_t = flux_linkage * self.polePairVal.value() * 1.5

        self.KtMotorVal.setValue(K_t)
        self.KtOutputVal.setValue(self.KtMotorVal.value() * self.gearRatioVal.value())

    def LoadConfiguration(self, show_confirm=True):
        # TODO: Error check and retry?
        if(self.nomad_dev.load_configuration() == True):

            # Motor
            self.polePairVal.setValue(self.nomad_dev.motor_config.num_pole_pairs)
            self.KvVal.setValue(int(self.nomad_dev.motor_config.K_v))
            self.gearRatioVal.setValue(self.nomad_dev.motor_config.gear_ratio)
            self.KtMotorVal.setValue(self.nomad_dev.motor_config.K_t)
            self.KtOutputVal.setValue(self.nomad_dev.motor_config.K_t * self.nomad_dev.motor_config.gear_ratio)
            self.phaseResistanceVal.setValue(self.nomad_dev.motor_config.phase_resistance*1e3)
            #self.phaseInductanceVal.setValue(self.nomad_dev.motor_config.phase_inductance_d*1e6)
            self.phaseInductanceVal.setValue(self.nomad_dev.motor_config.phase_inductance_q*1e6)
            self.phaseOrderCombo.setCurrentIndex(self.nomad_dev.motor_config.phase_order)
            self.contCurrentVal.setValue(self.nomad_dev.motor_config.continuous_current_max)
            self.thermalTauVal.setValue(self.nomad_dev.motor_config.continuous_current_tau)

            # Encoder
            self.encoderCPRVal.setValue(self.nomad_dev.encoder_config.cpr)
            self.electricalOffsetVal.setValue(self.nomad_dev.encoder_config.offset_elec)
            self.mechanicalOffsetVal.setValue(self.nomad_dev.encoder_config.offset_mech)

            # Controller
            self.pwmFreqVal.setValue(self.nomad_dev.controller_config.pwm_freq)
            self.loopFreqDividerVal.setValue(self.nomad_dev.controller_config.foc_ccl_divider)
            self.overmodulationPercentageVal.setValue(self.nomad_dev.controller_config.overmodulation*100.0)
            self.loopBandwidthVal.setValue(self.nomad_dev.controller_config.current_bandwidth)
            self.loopGainDAxisVal.setValue(self.nomad_dev.controller_config.k_d)
            self.loopGainQAxisVal.setValue(self.nomad_dev.controller_config.k_q)
            self.integratorGainDAxisVal.setValue(self.nomad_dev.controller_config.k_i_d)
            self.integratorGainQAxisVal.setValue(self.nomad_dev.controller_config.k_i_q)
            #self.refFilterCoefficientVal.setValue(self.nomad_dev.controller_config.alpha)
            
            self.posGainMaxVal.setValue(self.nomad_dev.controller_config.K_p_max)
            self.velGainMaxVal.setValue(self.nomad_dev.controller_config.K_d_max)

            self.posLimitMinVal.setValue(self.nomad_dev.controller_config.pos_limit_min)
            self.posLimitMaxVal.setValue(self.nomad_dev.controller_config.pos_limit_max)
            
            self.posLimitPGainVal.setValue(self.nomad_dev.controller_config.K_p_limit)
            self.posLimitIGainVal.setValue(self.nomad_dev.controller_config.K_i_limit)
            self.posLimitDGainVal.setValue(self.nomad_dev.controller_config.K_d_limit)
            self.velLimitVal.setValue(self.nomad_dev.controller_config.velocity_limit)
            
            self.currentLimitVal.setValue(self.nomad_dev.controller_config.current_limit)
            self.torqueLimitVal.setValue(self.nomad_dev.controller_config.torque_limit)

            if(show_confirm):
                msgBox = QtWidgets.QMessageBox()
                msgBox.setIcon(QtWidgets.QMessageBox.Information)
                msgBox.setText("Configuration Loaded Successfully.")
                msgBox.setWindowTitle("Nomad BLDC")
                msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
                msgBox.exec()

    def SaveConfiguration(self):

        # Must be idle
        if(self.idle != True): # Can only save when idle.  Will prevent blowups
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Critical)
            msgBox.setText("ERROR:  Controller must be IDLE to save.")
            msgBox.setWindowTitle("Nomad BLDC")
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec()
            return

        # Update Motor Parameters
        self.nomad_dev.motor_config.num_pole_pairs = self.polePairVal.value()
        self.nomad_dev.motor_config.continuous_current_max = self.contCurrentVal.value()
        self.nomad_dev.motor_config.continuous_current_tau = self.thermalTauVal.value()
        self.nomad_dev.motor_config.K_v = self.KvVal.value()
        self.nomad_dev.motor_config.gear_ratio = self.gearRatioVal.value()
        self.nomad_dev.motor_config.K_t = self.KtMotorVal.value()
        self.nomad_dev.motor_config.K_t_out = self.KtOutputVal.value()
        self.nomad_dev.motor_config.phase_resistance = self.phaseResistanceVal.value()*1e-3
        self.nomad_dev.motor_config.phase_inductance_d = self.phaseInductanceVal.value()*1e-6
        self.nomad_dev.motor_config.phase_inductance_q = self.phaseInductanceVal.value()*1e-6
        self.nomad_dev.motor_config.phase_order = self.phaseOrderCombo.currentIndex()

        #TODO: This should be based on the success of the calibration procedure.  
        self.nomad_dev.motor_config.calibrated = 1 # Calibrated

        # Update Controller Parameters
        self.nomad_dev.controller_config.k_d = self.loopGainDAxisVal.value()
        self.nomad_dev.controller_config.k_q = self.loopGainQAxisVal.value()
        self.nomad_dev.controller_config.k_i_d = self.integratorGainDAxisVal.value()
        self.nomad_dev.controller_config.k_i_q = self.integratorGainQAxisVal.value()
        #self.nomad_dev.controller_config.alpha = self.refFilterCoefficientVal.value()
        self.nomad_dev.controller_config.overmodulation = self.overmodulationPercentageVal.value() / 100.0

        self.nomad_dev.controller_config.velocity_limit = self.velLimitVal.value()
        self.nomad_dev.controller_config.pos_limit_min = self.posLimitMinVal.value()
        self.nomad_dev.controller_config.pos_limit_max = self.posLimitMaxVal.value()
        self.nomad_dev.controller_config.torque_limit = self.torqueLimitVal.value()
        self.nomad_dev.controller_config.current_limit = self.currentLimitVal.value()
        self.nomad_dev.controller_config.current_bandwidth = self.loopBandwidthVal.value()
        self.nomad_dev.controller_config.K_d_max = self.velGainMaxVal.value()
        self.nomad_dev.controller_config.K_p_max = self.posGainMaxVal.value()

        self.nomad_dev.controller_config.K_p_limit = self.posLimitPGainVal.value()
        self.nomad_dev.controller_config.K_i_limit = self.posLimitIGainVal.value()
        self.nomad_dev.controller_config.K_d_limit = self.posLimitDGainVal.value()

        self.nomad_dev.controller_config.pwm_freq = self.pwmFreqVal.value()
        self.nomad_dev.controller_config.foc_ccl_divider = self.loopFreqDividerVal.value()

        # Update Encoder Parameters
        self.nomad_dev.encoder_config.cpr = self.encoderCPRVal.value()
        self.nomad_dev.encoder_config.offset_elec = self.electricalOffsetVal.value()
        self.nomad_dev.encoder_config.offset_mech = self.mechanicalOffsetVal.value()

        if(self.nomad_dev.save_configuration() != True):
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Critical)
            msgBox.setText("Configuration Save FAILED.")
            msgBox.setWindowTitle("Nomad BLDC")
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec()
        else:
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Information)
            msgBox.setText("Configuration Saved Successfully.\nPress OK to Reset")
            msgBox.setWindowTitle("Nomad BLDC")
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec()    
            self.RestartDevice()


    def CalibrateDevice(self):
        self.nomad_dev.calibrate_motor()

    def RestartDevice(self):
        self.nomad_dev.restart_device() 
    
    def StartVoltageControl(self):
        self.nomad_dev.start_voltage_control()
    
    def StartTorqueControl(self):
        self.nomad_dev.start_torque_control()
    
    def StartCurrentControl(self):
        self.nomad_dev.start_current_control()

    def EnterIdleMode(self):
        self.nomad_dev.enter_idle_mode()

    def SetVoltageSetPoint(self):
        max_voltage = 10.0
        v_q = max_voltage * self.voltageControlSlider.value()/1000.0
        self.nomad_dev.set_voltage_setpoint(0, v_q) 
    
    def SetTorqueSetPoint(self):
        self.nomad_dev.set_torque_setpoint(self.k_p_spin.value(), self.k_d_spin.value(), self.pos_spin.value(), self.vel_spin.value(), self.torqueFF_spin.value()) 
        # self.TorqueOut.setText(f"Torque Out: {self.torqueFF_spin.value()}")
        # self.CurrentOut.setText(f"Iq: {self.torqueFF_spin.value() / (self.nomad_dev.motor_config.gear_ratio * self.nomad_dev.motor_config.K_t)}")
        # arm = 0.490
        # force = (self.torqueFF_spin.value()) / arm
        # self.scaleValue.setText(f"{1000*(force/9.81)}")

    def AutoComputeControllerGains(self):

        pwm_freq = self.pwmFreqVal.value()
        control_loop_freq = pwm_freq / self.loopFreqDividerVal.value()
        control_loop_period = 1.0 / control_loop_freq
        current_bandwidth = self.loopBandwidthVal.value()
        R_phase = self.phaseResistanceVal.value()*1e-3
        L_phase = self.phaseInductanceVal.value()*1e-6

        crossover_freq = current_bandwidth * control_loop_period * 2 * math.pi
        k_i = 1 - math.exp(-R_phase * control_loop_period / L_phase)
        k = R_phase * ((crossover_freq) / k_i)

        k_d = k_q = k
        k_i_d = k_i_q = k_i
        #alpha = 1.0 - 1.0 / (1.0 - control_loop_period * current_bandwidth * 2.0 * math.pi)

        # Update UI
        self.loopGainDAxisVal.setValue(k_d)
        self.loopGainQAxisVal.setValue(k_q)
        self.integratorGainDAxisVal.setValue(k_i_d)
        self.integratorGainQAxisVal.setValue(k_i_q)
        #self.refFilterCoefficientVal.setValue(alpha)


    def ShowEncoderEccentricity(self):
      #  print(f"Config: {self.nomad_dev.encoder_config.offset_lut}"

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOption('antialias', True)
        
        x = np.linspace(0, 360, 128)

        pw = pg.plot(x=x, y=self.nomad_dev.encoder_config.offset_lut, pen='k', symbolBrush=(255,0,0), symbolPen='k')#, symbol='o')  # plot x vs y in red
        pw.showGrid(x=True, y=True)
        #pw.setMouseEnabled(False)
        pw.setMenuEnabled(False)

        pw.setLabel('left', 'Offset', 'Encoder Counts')
        pw.setLabel('bottom', 'Rotor Angle', 'deg')

    def closeEvent(self, event):
        self.nomad_dev.disconnect()
        close_event.set()

    def UpdateSlot(self, stats):
        if(stats is not None): # Update Stats
            time_str = time.strftime("%Hh %Mm %Ss", time.gmtime(stats.uptime))
            self.uptimeLabel.setText(f"Up Time: " + time_str)
            self.busVoltageLabel.setText("V<sub>(bus)</sub>: <b>{:0.2f} V</b>".format(stats.voltage_bus))
            self.controllerStatusLabel.setText(f"Controller Status: <b>{Mode_Map[stats.control_status]}</b>")
            self.busCurrentLabel.setText("I<sub>(bus)</sub>: <b>{:0.2f} A</b>".format(stats.current_bus))
            self.fetTempLabel.setText("FET Temp: <b>{:0.1f}</b> C".format(stats.fet_temp))
            self.motorTempLabel.setText("Motor Temp: <b>{:0.1f}</b> C".format(stats.motor_temp))
            self.controllerFaultLabel.setText("Fault: <b>None</b>")
            if(Mode_Map[stats.control_status] == 'IDLE'):
                self.idle = True
            else:
                self.idle = False
        
    def UpdateState(self, state):
        if(state == True):
            motor_state = self.nomad_dev.motor_state
            controller_state = self.nomad_dev.controller_state
            encoder_state = self.nomad_dev.encoder_state
            controller_config = self.nomad_dev.controller_config
            #print(controller_state)
            max_current = self.nomad_dev.controller_config.current_limit
            I_d, I_q = self.nomad_dev.dq0(motor_state.theta_elec, motor_state.I_a, motor_state.I_b, motor_state.I_c)
            #print(I_q)
           # print(state)
            self.idProgressVal.setFormat("I_d: {:0.2f} A".format(I_d))
            self.idProgressVal.setValue(int(abs(I_d/max_current)*100))

            self.iqProgressVal.setFormat("I_q: {:0.2f} A".format(I_q))
            self.iqProgressVal.setValue(int(abs(I_q/max_current)*100))

            self.vdProgressVal.setFormat("V_d: {:0.2f} V".format(controller_state.V_d))
            self.vdProgressVal.setValue(int(abs(controller_state.V_d/20.0)*100))

            self.vqProgressVal.setFormat("V_q: {:0.2f} V".format(controller_state.V_q))
            self.vqProgressVal.setValue(int(abs(controller_state.V_q/20.0)*100))


            torque = (I_q * self.nomad_dev.motor_config.K_t * self.nomad_dev.motor_config.gear_ratio)
            self.torqueProgressVal.setFormat("Torque: {:0.2f} N*m".format(torque))
            self.torqueProgressVal.setValue(int(abs((torque)/(max_current*self.nomad_dev.motor_config.K_t*self.nomad_dev.motor_config.gear_ratio))*100))

          #  print(self.nomad_dev.motor_state)
          #  print(item[1])

            #print(state)

    def UpdateLog(self, log_info):
        
        # Append to terminal
        self.logTerminalEdit.append(log_info)

        # Autoscroll?
        if(self.autoScrollTerminalCheck.isChecked()):
            self.logTerminalEdit.ensureCursorVisible()

    def UpdatePlots(self):
            
        if(self.nomad_dev.controller_state is None): 
            return
        self.duty = self.nomad_dev.controller_state.dtc_A #self.duty + 0.05 * self.dir #np.random.uniform(low=0.0, high=1.0)
        if(self.duty is None):
            return
       # self.duty = min(self.duty, 0.8)
       # if(self.duty >= 0.8):
        #    self.dir = self.dir * -1
            
        self.data1 = np.zeros(500)
        self.duty_width = int(self.duty * 500)

        #print(250-self.duty_width // 2)
        #print(self.duty_width)
        #self.data1[250-self.duty_width // 2:self.duty_width] = 1
        duty_l = 250-self.duty_width // 2
        self.data1[duty_l:duty_l + self.duty_width] = 1

        #self.data1[:-1] = self.data1[1:]  # shift data in the array one sample left
                            # (see also: np.roll)
        #self.data1[-1] = np.random.normal()

        #self.sample_ptr1 += 1
        self.curve2.setData(self.data1)
        #self.curve2.setPos(self.sample_ptr1, 0)
    def DoLoadTest(self):
        print("Load Test")
        self.EnterIdleMode()
        time.sleep(2)

        t_calc = TorqueCalibrater(self.nomad_dev)
        self.threadpool.start(t_calc)

        # Connect to Scale
    def DoThermalTest(self):
        print("Thermal Test")
        self.nomad_dev.start_current_control()
        self.nomad_dev.set_current_setpoint(0, 12) 


if __name__ == '__main__':

    app = QtWidgets.QApplication([])
    window = NomadBLDCGUI()
    sys.exit(app.exec_())




