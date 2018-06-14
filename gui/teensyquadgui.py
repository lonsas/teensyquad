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
        fields = [g_K.get(), g_Ti.get(), g_Td.get(), gy_K.get(), gy_Ti.get(), gy_Td.get(), a_K.get(), a_Ti.get(), a_Td.get()]
        field_data = [float(x) for x in fields]

    def loadPID(self, gyro_pids, angle_pids):
        self.gyro_pids = gyro_pids
        self.angle_pids = angle_pids
#        self.g_K.set(


    def initialize(self, dataserial):
        self.dataserial = dataserial
        self.connect(dataserial, dataserial.signal, self.updateData)
        self.connect(self.saveButton, QtCore.SIGNAL("clicked()"), dataserial.savePID)
        self.connect(self.loadButton, QtCore.SIGNAL("clicked()"), dataserial.loadPID)
        self.connect(dataserial, dataserial.loadPIDDone, self.loadPID)

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
        self.signal = QtCore.SIGNAL("dataReady")
        self.loadPIDDone = QtCore.SIGNAL("loadPidDone")


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
        gyro_pid = struct.pack("cxxxxxxxdddddddddddddddddddddddd",
                Commands.USB_WRITE_GYRO_PID,
                parameters[0:24]
                )
        gyro_data = cobs.encode(gyro_pid)
        self.teensy.write(gyro_data)
    def loadPID():
        pass

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
            self.emit(self.signal, sensorStruct.parse(data))
        elif(command == self.Commands.USB_LOG_STATS.value):
            print(statsStruct.parse(data).dt)
        elif(command == self.Commands.USB_LOG_RECEIVER.value):
            print(receiverStruct.parse(data).auxb)


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
