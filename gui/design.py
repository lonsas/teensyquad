# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'teensyquad.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1034, 615)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.formLayout = QtGui.QFormLayout(self.centralwidget)
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.measurementPlot = PlotWidget(self.centralwidget)
        self.measurementPlot.setMinimumSize(QtCore.QSize(1016, 192))
        self.measurementPlot.setObjectName(_fromUtf8("measurementPlot"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.SpanningRole, self.measurementPlot)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label)
        self.sampleTimeLabel = QtGui.QLabel(self.centralwidget)
        self.sampleTimeLabel.setMaximumSize(QtCore.QSize(40, 16777215))
        self.sampleTimeLabel.setAutoFillBackground(False)
        self.sampleTimeLabel.setStyleSheet(_fromUtf8("background-color: rgb(255, 255, 255);"))
        self.sampleTimeLabel.setTextFormat(QtCore.Qt.PlainText)
        self.sampleTimeLabel.setObjectName(_fromUtf8("sampleTimeLabel"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.sampleTimeLabel)
        self.outputPlot = PlotWidget(self.centralwidget)
        self.outputPlot.setMinimumSize(QtCore.QSize(0, 192))
        self.outputPlot.setObjectName(_fromUtf8("outputPlot"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.SpanningRole, self.outputPlot)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1034, 19))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Teensyquad gui", None))
        self.label.setText(_translate("MainWindow", "dt", None))
        self.sampleTimeLabel.setText(_translate("MainWindow", "12345", None))

from pyqtgraph import PlotWidget
