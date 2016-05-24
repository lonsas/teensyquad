from PyQt4 import QtGui
from PyQt4.QtCore import QThread
import sys
import numpy as np
from PyQt4 import QtCore
import design
import serial
from serial.tools import list_ports
from construct import Struct, UBInt8, SBInt16, UBInt32, SLInt16, ULInt32
from pyqtgraph.ptime import time


sensorStruct = Struct("datastruct",
                    SLInt16("accx"),
                    SLInt16("accy"),
                    SLInt16("accz"),
                    SLInt16("gyrox"),
                    SLInt16("gyroy"),
                    SLInt16("gyroz"),
                    ULInt32("dt"))


class TeensyGUI(QtGui.QMainWindow, design.Ui_MainWindow):
    def __init__(self, parent=None):
        super(TeensyGUI, self).__init__(parent)
        self.setupUi(self)
        self.measurementPlot.setYRange(-2**14, 2**14)
        self.accxCurve = self.measurementPlot.plot(pen=(0, 6))
        self.accx = [0]
        self.accyCurve = self.measurementPlot.plot(pen=(1, 6))
        self.accy = [0]
        self.acczCurve = self.measurementPlot.plot(pen=(2, 6))
        self.accz = [0]
        self.gyroxCurve = self.measurementPlot.plot(pen=(3, 6))
        self.gyrox = [0]
        self.gyroyCurve = self.measurementPlot.plot(pen=(4, 6))
        self.gyroy = [0]
        self.gyrozCurve = self.measurementPlot.plot(pen=(5, 6))
        self.gyroz = [0]
        self.startTime = time()
        self.plottime = [0]
        self.dt = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlots)
        self.timer.start(0)




        #self.pushButtonA.clicked.connect(simpleAction)
        #self.curve = self.plotA.plot(pen=(255,0,0))


    def initialize(self, dataserial):
        self.dataserial = dataserial
        self.connect(dataserial, dataserial.signal, self.updateData)

    def updateData(self, data):
        self.plottime.append(time() - self.startTime)
        self.accx.append(data.accx)
        self.accy.append(data.accy)
        self.accz.append(data.accz)
        self.gyrox.append(data.gyrox)
        self.gyroy.append(data.gyroy)
        self.gyroz.append(data.gyroz)
        self.dt = data.dt


    def updatePlots(self):
        self.accxCurve.setData(self.plottime, self.accx)
        self.accyCurve.setData(self.plottime, self.accy)
        self.acczCurve.setData(self.plottime, self.accz)
        self.gyroxCurve.setData(self.plottime, self.gyrox)
        self.gyroyCurve.setData(self.plottime, self.gyroy)
        self.gyrozCurve.setData(self.plottime, self.gyroz)
        self.sampleTimeLabel.setText(str(self.dt))
        self.measurementPlot.setXRange(self.plottime[-1] - 10, self.plottime[-1])



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
            if port[1].startswith("Teensy") or port[0] == '/dev/ttyACM0':
                teensy_port = port
        if teensy_port:
            return teensy_port

    def close(self):
        if self.teensy.isOpen():
            self.teensy.close()

    def run(self):
        while(True):
            sensors = sensorStruct.parse(self.teensy.read(sensorStruct.sizeof()))
            self.emit(self.signal, sensors)








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