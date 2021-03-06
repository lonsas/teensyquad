# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'teensyquad.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
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
        MainWindow.resize(1052, 783)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tab = QtGui.QWidget()
        self.tab.setObjectName(_fromUtf8("tab"))
        self.layoutWidget = QtGui.QWidget(self.tab)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 10, 107, 22))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_4.setMargin(0)
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label = QtGui.QLabel(self.layoutWidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout_4.addWidget(self.label)
        self.sampleTimeLabel = QtGui.QLabel(self.layoutWidget)
        self.sampleTimeLabel.setMaximumSize(QtCore.QSize(40, 16777215))
        self.sampleTimeLabel.setAutoFillBackground(False)
        self.sampleTimeLabel.setStyleSheet(_fromUtf8("background-color: rgb(255, 255, 255);"))
        self.sampleTimeLabel.setTextFormat(QtCore.Qt.PlainText)
        self.sampleTimeLabel.setObjectName(_fromUtf8("sampleTimeLabel"))
        self.horizontalLayout_4.addWidget(self.sampleTimeLabel)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.outputPlot = PlotWidget(self.tab)
        self.outputPlot.setGeometry(QtCore.QRect(0, 280, 1034, 217))
        self.outputPlot.setMinimumSize(QtCore.QSize(0, 0))
        self.outputPlot.setObjectName(_fromUtf8("outputPlot"))
        self.autoPanButton = QtGui.QCheckBox(self.tab)
        self.autoPanButton.setGeometry(QtCore.QRect(0, 40, 1034, 22))
        self.autoPanButton.setObjectName(_fromUtf8("autoPanButton"))
        self.measurementPlot = PlotWidget(self.tab)
        self.measurementPlot.setGeometry(QtCore.QRect(0, 60, 1034, 216))
        self.measurementPlot.setMinimumSize(QtCore.QSize(0, 0))
        self.measurementPlot.setAutoFillBackground(False)
        self.measurementPlot.setStyleSheet(_fromUtf8(""))
        self.measurementPlot.setObjectName(_fromUtf8("measurementPlot"))
        self.outputPlot.raise_()
        self.autoPanButton.raise_()
        self.measurementPlot.raise_()
        self.layoutWidget.raise_()
        self.outputPlot.raise_()
        self.autoPanButton.raise_()
        self.measurementPlot.raise_()
        self.label.raise_()
        self.sampleTimeLabel.raise_()
        self.tabWidget.addTab(self.tab, _fromUtf8(""))
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName(_fromUtf8("tab_2"))
        self.g_K = QtGui.QTextEdit(self.tab_2)
        self.g_K.setGeometry(QtCore.QRect(140, 70, 101, 31))
        self.g_K.setObjectName(_fromUtf8("g_k"))
        self.g_Ti = QtGui.QTextEdit(self.tab_2)
        self.g_Ti.setGeometry(QtCore.QRect(140, 100, 101, 31))
        self.g_Ti.setObjectName(_fromUtf8("g_ti"))
        self.g_Td = QtGui.QTextEdit(self.tab_2)
        self.g_Td.setGeometry(QtCore.QRect(140, 130, 101, 31))
        self.g_Td.setObjectName(_fromUtf8("g_Td"))
        self.label_2 = QtGui.QLabel(self.tab_2)
        self.label_2.setGeometry(QtCore.QRect(70, 50, 62, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.tab_2)
        self.label_3.setGeometry(QtCore.QRect(120, 80, 16, 16))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.tab_2)
        self.label_4.setGeometry(QtCore.QRect(120, 110, 16, 16))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.tab_2)
        self.label_5.setGeometry(QtCore.QRect(120, 140, 16, 16))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gy_K = QtGui.QTextEdit(self.tab_2)
        self.gy_K.setGeometry(QtCore.QRect(140, 190, 101, 31))
        self.gy_K.setObjectName(_fromUtf8("gy_K"))
        self.label_6 = QtGui.QLabel(self.tab_2)
        self.label_6.setGeometry(QtCore.QRect(120, 260, 16, 16))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.label_7 = QtGui.QLabel(self.tab_2)
        self.label_7.setGeometry(QtCore.QRect(120, 200, 16, 16))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.gy_Ti = QtGui.QTextEdit(self.tab_2)
        self.gy_Ti.setGeometry(QtCore.QRect(140, 220, 101, 31))
        self.gy_Ti.setObjectName(_fromUtf8("gy_Ti"))
        self.gy_Td = QtGui.QTextEdit(self.tab_2)
        self.gy_Td.setGeometry(QtCore.QRect(140, 250, 101, 31))
        self.gy_Td.setObjectName(_fromUtf8("gy_Td"))
        self.label_8 = QtGui.QLabel(self.tab_2)
        self.label_8.setGeometry(QtCore.QRect(60, 170, 62, 17))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.label_9 = QtGui.QLabel(self.tab_2)
        self.label_9.setGeometry(QtCore.QRect(120, 230, 16, 16))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.a_K = QtGui.QTextEdit(self.tab_2)
        self.a_K.setGeometry(QtCore.QRect(140, 320, 101, 31))
        self.a_K.setObjectName(_fromUtf8("a_K"))
        self.label_10 = QtGui.QLabel(self.tab_2)
        self.label_10.setGeometry(QtCore.QRect(120, 390, 16, 16))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.label_11 = QtGui.QLabel(self.tab_2)
        self.label_11.setGeometry(QtCore.QRect(120, 330, 16, 16))
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.a_Ti = QtGui.QTextEdit(self.tab_2)
        self.a_Ti.setGeometry(QtCore.QRect(140, 350, 101, 31))
        self.a_Ti.setObjectName(_fromUtf8("a_Ti"))
        self.a_Td = QtGui.QTextEdit(self.tab_2)
        self.a_Td.setGeometry(QtCore.QRect(140, 380, 101, 31))
        self.a_Td.setObjectName(_fromUtf8("a_Td"))
        self.label_12 = QtGui.QLabel(self.tab_2)
        self.label_12.setGeometry(QtCore.QRect(70, 300, 62, 17))
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.label_13 = QtGui.QLabel(self.tab_2)
        self.label_13.setGeometry(QtCore.QRect(120, 360, 16, 16))
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.loadButton = QtGui.QPushButton(self.tab_2)
        self.loadButton.setGeometry(QtCore.QRect(50, 440, 92, 27))
        self.loadButton.setObjectName(_fromUtf8("loadButton"))
        self.saveButton = QtGui.QPushButton(self.tab_2)
        self.saveButton.setGeometry(QtCore.QRect(160, 440, 92, 27))
        self.saveButton.setObjectName(_fromUtf8("saveButton"))
        self.tabWidget.addTab(self.tab_2, _fromUtf8(""))
        self.verticalLayout.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1052, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Teensyquad gui", None))
        self.label.setText(_translate("MainWindow", "dt", None))
        self.sampleTimeLabel.setText(_translate("MainWindow", "12345", None))
        self.autoPanButton.setText(_translate("MainWindow", "Auto Pan", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Graph", None))
        self.label_2.setText(_translate("MainWindow", "Gyro", None))
        self.label_3.setText(_translate("MainWindow", "K", None))
        self.label_4.setText(_translate("MainWindow", "Ti", None))
        self.label_5.setText(_translate("MainWindow", "Td", None))
        self.label_6.setText(_translate("MainWindow", "Td", None))
        self.label_7.setText(_translate("MainWindow", "K", None))
        self.label_8.setText(_translate("MainWindow", "Gyro Yaw", None))
        self.label_9.setText(_translate("MainWindow", "Ti", None))
        self.label_10.setText(_translate("MainWindow", "Td", None))
        self.label_11.setText(_translate("MainWindow", "K", None))
        self.label_12.setText(_translate("MainWindow", "Angle", None))
        self.label_13.setText(_translate("MainWindow", "Ti", None))
        self.loadButton.setText(_translate("MainWindow", "Load", None))
        self.saveButton.setText(_translate("MainWindow", "Save", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "PID", None))

from pyqtgraph import PlotWidget
