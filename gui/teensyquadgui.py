from PyQt4 import QtGui
from PyQt4.QtCore import QThread
import sys
import numpy as np
from PyQt4 import QtCore
import design
import serial
from serial.tools import list_ports
from construct import *
from pyqtgraph.ptime import time
import pyqtgraph
from cobs import cobs
from enum import Enum

sensorStruct = AlignedStruct(8, "command" / Int8ul,
                      "gyrox" / Float64l,
                      "gyroy" / Float64l,
                      "gyroz" / Float64l,
                      "accx" / Float64l,
                      "accy" / Float64l,
                      "accz" / Float64l)
statsStruct = AlignedStruct(4, "command" / Int8ul,
                      "dt" / Int32ul)
receiverStruct = AlignedStruct(8, "command" / Int8ul,
                      "roll" / Float64l,
                      "pitch" / Float64l,
                      "throttle" / Float64l,
                      "yaw" / Float64l,
                      "auxa" / Float64l,
                      "auxb" / Float64l);

command_fmt8 = "cxxxxxxx"
command_fmt4 = "cxxx"
pid_fmt = command_fmt8 + "dddddddddddddddddddddddd"
sensor_fmt = command_fmt8 + "dddddd"
receiver_fmt = command_fmt8 + "dddddd"
stats_fmt = command_fmt4 + "I"


pyqtgraph.setConfigOption('background', 'w')
pyqtgraph.setConfigOption('foreground', 'k')
class TeensyGUI(QtGui.QMainWindow, design.Ui_MainWindow):
    def __init__(self, parent=None):
        super(TeensyGUI, self).__init__(parent)
        self.setupUi(self)
        self.setupGraphs()

    def setupGraphs(self):
        #self.measurementPlot.setClipToView(True)
        self.measurementPlot.setLimits(yMax=2**15, yMin=-2**15)

        self.accxCurve = self.measurementPlot.plot(pen='r')
        self.accx = []
        self.accyCurve = self.measurementPlot.plot(pen='g')
        self.accy = []
        self.acczCurve = self.measurementPlot.plot(pen='b')
        self.accz = []
        self.gyroxCurve = self.outputPlot.plot(pen='r')
        self.gyrox = []
        self.gyroyCurve = self.outputPlot.plot(pen='g')
        self.gyroy = []
        self.gyrozCurve = self.outputPlot.plot(pen='b')
        self.gyroz = []


        #self.pitchCurve = self.outputPlot.plot(pen=(0, 3))
        #self.pitch = []
        #self.rollCurve = self.outputPlot.plot(pen=(1, 3))
        #self.roll = []
        #self.yawCurve = self.outputPlot.plot(pen=(2, 3))
        #self.yaw = []


        self.startTime = time()
        self.plottime = []
        self.dt = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlots)
        self.timer.start(50)

    def savePID(self):
        fields = [self.g_K.toPlainText(),
                  self.g_Ti.toPlainText(),
                  self.g_Td.toPlainText(),
                  self.gy_K.toPlainText(),
                  self.gy_Ti.toPlainText(),
                  self.gy_Td.toPlainText(),
                  self.a_K.toPlainText(),
                  self.a_Ti.toPlainText(),
                  self.a_Td.toPlainText()]
        field_data = [float(x) for x in fields]

        # Gyro roll and pitch pid
        self.gyro_pids[0] = gyro_pids[8] = field_data[0]
        self.gyro_pids[1] = gyro_pids[9] = field_data[1]
        self.gyro_pids[2] = gyro_pids[10] = field_data[2]

        self.gyro_pids[16] = field_data[3]
        self.gyro_pids[17] = field_data[4]
        self.gyro_pids[18] = field_data[5]

        # Angle pids
        self.angle_pids[0] = angle_pids[8] = angle_pids[16] =  field_data[6]
        self.angle_pids[1] = angle_pids[9] = angle_pids[17] = field_data[7]
        self.angle_pids[2] = angle_pids[10] = angle_pids[18] = field_data[8]

        dataserial.savePID(self.gyro_pids, self.angle_pids)

    def loadAnglePID(self, angle_pids):
        self.angle_pids = angle_pids

        self.a_K.set(angle_pids[0])
        self.a_Ti.set(angle_pids[1])
        self.a_Td.set(angle_pids[2])

    def loadGyroPID(self, gyro_pids):
        self.gyro_pids = gyro_pids

        self.g_K.set(gyro_pids[0])
        self.g_Ti.set(gyro_pids[1])
        self.g_Td.set(gyro_pids[2])

        self.gy_K.set(gyro_pids[16])
        self.gy_Ti.set(gyro_pids[17])
        self.gy_Td.set(gyro_pids[18])

    def showDt(self, dt):
        self.sampleTimeLabel.setText(dt)

    def initialize(self, dataserial):
        # Serial events
        self.dataserial = dataserial
        self.connect(dataserial, dataserial.log_signal, self.updateData)
        self.connect(dataserial, dataserial.load_gyro_pid_done, self.loadGyroPID)
        self.connect(dataserial, dataserial.load_angle_pid_done, self.loadAnglePID)
        self.connect(dataserial, dataserial.dt_signal, self.showDt)

        # Gui events
        self.connect(self.saveButton, QtCore.SIGNAL("clicked()"), self.savePID)
        self.connect(self.loadButton, QtCore.SIGNAL("clicked()"), dataserial.loadPID)



    def updateData(self, data):
        self.accx.append(data.accx)
        self.accy.append(data.accy)
        self.accz.append(data.accz)
        self.gyrox.append(data.gyrox)
        self.gyroy.append(data.gyroy)
        self.gyroz.append(data.gyroz)
#        self.pitch.append(data.pitch)
#        self.roll.append(data.roll)
#        self.yaw.append(data.yaw)
#        self.plottime.append(data.t/1000000)
#        self.dt = data.dt


    def updatePlots(self):
        length = 10**5 # Not too much data to plot, clipping is horrible
        self.plottime = [i for i in range(len(self.accx[-length:]))]
        self.accxCurve.setData(self.plottime, self.accx[-length:])
        self.accyCurve.setData(self.plottime, self.accy[-length:])
        self.acczCurve.setData(self.plottime, self.accz[-length:])
        self.gyroxCurve.setData(self.plottime, self.gyrox[-length:])
        self.gyroyCurve.setData(self.plottime, self.gyroy[-length:])
        self.gyrozCurve.setData(self.plottime, self.gyroz[-length:])

#        self.rollCurve.setData(self.plottime[-length:], self.pitch[-length:])
#        self.pitchCurve.setData(self.plottime[-length:], self.roll[-length:])
#        self.yawCurve.setData(self.plottime[-length:], self.yaw[-length:])


#        self.sampleTimeLabel.setText(str(self.dt))
        if(self.autoPanButton.isChecked()):
            if self.xrange == None:
                [[xmin, xmax], [ymin, ymax]] = self.measurementPlot.viewRange()
                self.xrange = xmax-xmin
            print(self.xrange)
            #self.measurementPlot.setLimits(xMax=self.plottime[-1])
            self.measurementPlot.setXRange(self.plottime[-1] - self.xrange, self.plottime[-1])
        else:
            self.xrange = None


class TeensySerial(QThread):
    class Commands(Enum):
        USB_INVALID = 0
        USB_LOG_START = 1
        USB_LOG_STOP = 2
        USB_LOG_SENSOR = 3
        USB_WRITE_GYRO_PID = 4
        USB_READ_GYRO_PID = 5
        USB_WRITE_ANGLE_PID = 6
        USB_ANGLE_ANGLE_PID = 7
        USB_LOG_STATS = 8
        USB_LOG_RECEIVER = 9

    def __init__(self):
        QThread.__init__(self)
        self.log_signal = QtCore.SIGNAL("log_data_ready")
        self.load_gyro_pid_done = QtCore.SIGNAL("load_gyro_pid_done")
        self.load_angle_pid_done = QtCore.SIGNAL("load_angle_pid_done")
        self.dt_signal = QtCore.SIGNAL("dt_signal")


    def getTeensyPort(self):
        """Discover where is Teensy."""
        ports_avaiable = list(list_ports.comports())
        teensy_port = tuple()
        for port in ports_avaiable:
            if port[1].startswith("Teensy") or port[0][:-1] == '/dev/ttyACM':
                teensy_port = port
        if teensy_port:
            return teensy_port

    def close(self):
        if self.teensy.isOpen():
            self.teensy.close()

    def savePID(self, gyro_parameters, angle_parameters):
        angle_pid = struct.pack(pid_fmt,
                Commands.USB_WRITE_ANGLE_PID_PID,
                gyro_parameters[0:24]
                )
        gyro_pid = struct.pack(pid_fmt,
                Commands.USB_WRITE_GYRO_PID,
                angle_parameters[0:24]
                )
        angle_data = cobs.encode(angle_pid)
        gyro_data = cobs.encode(gyro_pid)
        self.teensy.write(angle_data)
        self.teensy.write(gyro_data)

    def loadPID():
        self.teensy.write(cobs.encode(Commands.USB_READ_GYRO_PID))
        self.teensy.write(cobs.encode(Commands.USB_READ_ANGLE_PID))

    def setup(self):
        while(True):
            teensy_port = self.getTeensyPort()
            if(teensy_port):
                self.teensy = serial.Serial(teensy_port[0], 115200)
                break
            else:
                self.yieldCurrentThread()


    def run(self):
        self.setup()
        packet_valid = False
        max_size = 255
        data_buf = b''
        data_buf_idx = 0
        while(True):
            while(self.teensy.in_waiting == 0):
                self.yieldCurrentThread()
            data = self.teensy.read()
            if(data == b'\x00'):
                if(packet_valid and data_buf_idx > 0):
                    self.parseData(data_buf, data_buf_idx)
                    data_buf = b''
                    data_buf_idx = 0
                else:
                    packet_valid = True;
                    data_buf = b''
                    data_buf_idx = 0
            elif(packet_valid):
                data_buf += data
                data_buf_idx += 1
                if(data_buf_idx >= max_size):
                    packet_valid = False
                    print("Invalid packet")


    def parseData(self, data, length):
        data = cobs.decode(data)
        command = int(data[0])
        if(command == self.Commands.USB_LOG_SENSOR.value):
            self.emit(self.log_signal, sensorStruct.parse(data))
        elif(command == self.Commands.USB_LOG_STATS.value):
            self.emit(self.dt_signal, unpack(stat_fmt, data)[1])
        elif(command == self.Commands.USB_LOG_RECEIVER.value):
            print(receiverStruct.parse(data).auxb)
        elif(command == self.Commands.USB_READ_GYRO_PID):
            parameters = struct.unpack(pid_fmt, data)[1:]
            self.emit(self.load_gyro_pid_done, parameters)
        elif(command == self.Commands.USB_READ_ANGLE_PID):
            parameters = struct.unpack(pid_fmt, data)[1:]
            self.emit(self.load_angle_pid_done, parameters)


def main():
    app = QtGui.QApplication(sys.argv)

    dataSerial = TeensySerial()
    gui = TeensyGUI()
    gui.initialize(dataSerial)
    dataSerial.start()
    gui.show()
    app.exec_()


if __name__ == '__main__':
    main()
