from PyQt4 import QtGui
from PyQt4.QtCore import QThread
import sys
import numpy as np
from PyQt4 import QtCore
import design
import serial
from serial.tools import list_ports
from construct import Struct, UBInt8, SBInt16, UBInt32, SLInt16, ULInt32, ULInt8, SLInt32, LFloat32
from pyqtgraph.ptime import time
import pyqtgraph

sensorStruct = Struct("datastruct",
                    SLInt16("accx"),
                    SLInt16("accy"),
                    SLInt16("accz"),
                    SLInt16("gyrox"),
                    SLInt16("gyroy"),
                    SLInt16("gyroz"),

                    ULInt32("t"),
                    ULInt32("dt"),
                    LFloat32("pitch"),
                    LFloat32("roll"),
                    LFloat32("yaw"))

pyqtgraph.setConfigOption('background', 'w')
pyqtgraph.setConfigOption('foreground', 'k')
class TeensyGUI(QtGui.QMainWindow, design.Ui_MainWindow):
    def __init__(self, parent=None):
        super(TeensyGUI, self).__init__(parent)
        self.setupUi(self)
        self.setupGraphs()

    def setupGraphs(self):
        self.measurementPlot.setYRange(-2**14, 2**14)
        #self.measurementPlot.setClipToView(True)
        self.measurementPlot.setLimits(yMax=2**15, yMin=-2**15)

        self.accxCurve = self.measurementPlot.plot(pen=(0, 6))
        self.accx = []
        self.accyCurve = self.measurementPlot.plot(pen=(1, 6))
        self.accy = []
        self.acczCurve = self.measurementPlot.plot(pen=(2, 6))
        self.accz = []
        self.gyroxCurve = self.measurementPlot.plot(pen=(3, 6))
        self.gyrox = []
        self.gyroyCurve = self.measurementPlot.plot(pen=(4, 6))
        self.gyroy = []
        self.gyrozCurve = self.measurementPlot.plot(pen=(5, 6))
        self.gyroz = []
        self.gyroxCurve = self.measurementPlot.plot(pen=(3, 6))


        self.pitchCurve = self.outputPlot.plot(pen=(0, 3))
        self.pitch = []
        self.rollCurve = self.outputPlot.plot(pen=(1, 3))
        self.roll = []
        self.yawCurve = self.outputPlot.plot(pen=(2, 3))
        self.yaw = []


        self.startTime = time()
        self.plottime = []
        self.dt = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlots)
        self.timer.start(50)


    def initialize(self, dataserial):
        self.dataserial = dataserial
        self.connect(dataserial, dataserial.signal, self.updateData)

    def updateData(self, data):
        self.accx.append(data.accx)
        self.accy.append(data.accy)
        self.accz.append(data.accz)
        self.gyrox.append(data.gyrox)
        self.gyroy.append(data.gyroy)
        self.gyroz.append(data.gyroz)
        self.pitch.append(data.pitch)
        self.roll.append(data.roll)
        self.yaw.append(data.yaw)
        self.plottime.append(data.t/1000000)
        self.dt = data.dt


    def updatePlots(self):
        len = 10**5 # Not too much data to plot, clipping is horrible
        self.accxCurve.setData(self.plottime[-len:], self.accx[-len:])
        self.accyCurve.setData(self.plottime[-len:], self.accy[-len:])
        self.acczCurve.setData(self.plottime[-len:], self.accz[-len:])
        self.gyroxCurve.setData(self.plottime[-len:], self.gyrox[-len:])
        self.gyroyCurve.setData(self.plottime[-len:], self.gyroy[-len:])
        self.gyrozCurve.setData(self.plottime[-len:], self.gyroz[-len:])

        self.rollCurve.setData(self.plottime[-len:], self.pitch[-len:])
        self.pitchCurve.setData(self.plottime[-len:], self.roll[-len:])
        self.yawCurve.setData(self.plottime[-len:], self.yaw[-len:])


        self.sampleTimeLabel.setText(str(self.dt))
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
    def __init__(self, baudrate):
        QThread.__init__(self)
        self.teensy_port = self.getTeensyPort()
        self.teensy = serial.Serial(self.teensy_port[0], baudrate)
        self.signal = QtCore.SIGNAL("dataReady")

    def getTeensyPort(self):
        """Discover where is Teensy."""
        ports_avaiable = list(list_ports.comports())
        teensy_port = tuple()
        for port in ports_avaiable:
            print(port)
            if port[1].startswith("Teensy") or port[0] == '/dev/ttyACM0' or port[0] == '/dev/ttyACM1':
                teensy_port = port
        if teensy_port:
            return teensy_port

    def close(self):

        if self.teensy.isOpen():
            self.teensy.close()


    def run(self):
        while(True):
            while(self.teensy.in_waiting == 0):
                self.yieldCurrentThread()
            while(self.teensy.read() != b'\x01'):
                print("invalid data package 1")
                pass
            data = self.teensy.read(sensorStruct.sizeof())
            if(self.teensy.read() == b'\x02'): # This has probably been a valid package
                self.emit(self.signal, sensorStruct.parse(data))
            #else:
                #print("invalid data package 2")








def main():
    app = QtGui.QApplication(sys.argv)

    dataSerial = TeensySerial(115200)
    form = TeensyGUI()
    form.initialize(dataSerial)
    dataSerial.start()
    form.show()
    app.exec_()


if __name__ == '__main__':
    main()