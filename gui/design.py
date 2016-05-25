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
        MainWindow.resize(913, 599)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout_4.addWidget(self.label)
        self.sampleTimeLabel = QtGui.QLabel(self.centralwidget)
        self.sampleTimeLabel.setMaximumSize(QtCore.QSize(40, 16777215))
        self.sampleTimeLabel.setAutoFillBackground(False)
        self.sampleTimeLabel.setStyleSheet(_fromUtf8("background-color: rgb(255, 255, 255);"))
        self.sampleTimeLabel.setTextFormat(QtCore.Qt.PlainText)
        self.sampleTimeLabel.setObjectName(_fromUtf8("sampleTimeLabel"))
        self.horizontalLayout_4.addWidget(self.sampleTimeLabel)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.autoPanButton = QtGui.QCheckBox(self.centralwidget)
        self.autoPanButton.setObjectName(_fromUtf8("autoPanButton"))
        self.verticalLayout.addWidget(self.autoPanButton)
        self.measurementPlot = PlotWidget(self.centralwidget)
        self.measurementPlot.setMinimumSize(QtCore.QSize(0, 0))
        self.measurementPlot.setAutoFillBackground(False)
        self.measurementPlot.setStyleSheet(_fromUtf8(""))
        self.measurementPlot.setObjectName(_fromUtf8("measurementPlot"))
        self.verticalLayout.addWidget(self.measurementPlot)
        self.outputPlot = PlotWidget(self.centralwidget)
        self.outputPlot.setMinimumSize(QtCore.QSize(0, 0))
        self.outputPlot.setObjectName(_fromUtf8("outputPlot"))
        self.verticalLayout.addWidget(self.outputPlot)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 913, 19))
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
        self.autoPanButton.setText(_translate("MainWindow", "Auto Pan", None))

from pyqtgraph import PlotWidget
