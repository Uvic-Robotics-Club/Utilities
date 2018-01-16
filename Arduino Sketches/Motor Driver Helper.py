# -*- coding: utf-8 -*-
"""
Created on Fri Jan 05 18:30:48 2018

@author: joell
"""

execfile("Motor Driver Helper Callbacks.py")

import serial # import Serial Library
import serial.tools.list_ports as list_ports
print "the version of serial that I am using is {}".format(serial.VERSION)

import numpy as np
import sys
from PyQt4 import QtGui, uic

from matplotlib.backends import qt_compat
use_pyside = qt_compat.QT_API == qt_compat.QT_API_PYSIDE
if use_pyside:
    from PySide import QtGui, QtCore
else:
    from PyQt4 import QtGui, QtCore

import time
import string

print "Getting all of the available ports"
ports = list(list_ports.comports())
motors = [None, None, None, None, None, None, None]
motorCallbacks = [None, None, None, None, None, None, None]
XData = [None, None, None, None, None, None, None]
SetpointData = [None, None, None, None, None, None, None]
RpmData = [None, None, None, None, None, None, None]
colour = ["k", "b-", "g-", "r-", "c-", "m-", "k-"]
time.clock()
for (port,name,PID) in ports:
    print "Testing %s which is port: %s"%(name,port)
    sys.stdout.flush()
    if "CH340" in name:
        print "found the ardunio. opening comms"
        sys.stdout.flush()
        temparduinoData = serial.Serial(port, 9600) #Creating our serial object
        arduinoString = temparduinoData.readline()
        print arduinoString.strip()
        if "Motor:" in arduinoString:
            all=string.maketrans('','')
            nodigs=all.translate(all, string.digits)
            arduinoString=arduinoString.translate(all, nodigs)
            MotorNumber = int(arduinoString)
            motors[MotorNumber] = temparduinoData
            motorCallbacks[MotorNumber] = Callbacks(temparduinoData)
            SetpointData[MotorNumber] = [0]
            RpmData[MotorNumber] = [0]
            XData[MotorNumber] = [0]

class MainScreen(QtGui.QMainWindow):
    #overide the __init__ that qt designer creates so that I can add more functions
    def __init__(self):
        global motorCallbacks
        #overrides and loading the file
        super(self.__class__, self).__init__()
        uic.loadUi('Motor Driver Helper Screen.ui', self)
        if motorCallbacks[1] is not None:
            self.KpSpinner_1.valueChanged.connect(motorCallbacks[1].Kp)
            self.KiSpinner_1.valueChanged.connect(motorCallbacks[1].Ki)
            self.KdSpinner_1.valueChanged.connect(motorCallbacks[1].Kd)
            self.Setpoint_1.valueChanged.connect(motorCallbacks[1].Setpoint)
        else:
            self.groupBox_1.hide()
        if motorCallbacks[2] is not None:
            self.KpSpinner_2.valueChanged.connect(motorCallbacks[2].Kp)
            self.KiSpinner_2.valueChanged.connect(motorCallbacks[2].Ki)
            self.KdSpinner_2.valueChanged.connect(motorCallbacks[2].Kd)
            self.Setpoint_2.valueChanged.connect(motorCallbacks[2].Setpoint)
        else:
            self.groupBox_2.hide()
        if motorCallbacks[3] is not None:
            self.KpSpinner_3.valueChanged.connect(motorCallbacks[3].Kp)
            self.KiSpinner_3.valueChanged.connect(motorCallbacks[3].Ki)
            self.KdSpinner_3.valueChanged.connect(motorCallbacks[3].Kd)
            self.Setpoint_3.valueChanged.connect(motorCallbacks[3].Setpoint)
        else:
            self.groupBox_2.hide()
        if motorCallbacks[4] is not None:
            self.KpSpinner_4.valueChanged.connect(motorCallbacks[4].Kp)
            self.KiSpinner_4.valueChanged.connect(motorCallbacks[4].Ki)
            self.KdSpinner_4.valueChanged.connect(motorCallbacks[4].Kd)
            self.Setpoint_4.valueChanged.connect(motorCallbacks[4].Setpoint)
        else:
            self.groupBox_4.hide()
        if motorCallbacks[5] is not None:
            self.KpSpinner_5.valueChanged.connect(motorCallbacks[5].Kp)
            self.KiSpinner_5.valueChanged.connect(motorCallbacks[5].Ki)
            self.KdSpinner_5.valueChanged.connect(motorCallbacks[5].Kd)
            self.Setpoint_5.valueChanged.connect(motorCallbacks[5].Setpoint)
        else:
            self.groupBox_5.hide()
        if motorCallbacks[6] is not None:
            self.KpSpinner_6.valueChanged.connect(motorCallbacks[6].Kp)
            self.KiSpinner_6.valueChanged.connect(motorCallbacks[6].Ki)
            self.KdSpinner_6.valueChanged.connect(motorCallbacks[6].Kd)
            self.Setpoint_6.valueChanged.connect(motorCallbacks[6].Setpoint)
        else:
            self.groupBox_6.hide()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_figure)
        self.timer.start(200)

    def update_figure(self):
        global XData, SetpointData, RpmData, colour
        self.mplwidget.axes.cla()
        self.mplwidget.axes.hold(True)
        MotorNum = 0
        for i in range(len(motors)):
            if motors[i] is None:
                continue
            MotorNum += 1
            MotorData = motors[i].readline().strip().split(",")
            for j in range(len(MotorData)):
                MotorData[j] = float(MotorData[j])
            print "Motor {} : {}".format(i, MotorData)
            SetpointData[i].append(MotorData[2])
            RpmData[i].append(MotorData[0])
            XData[i].append(time.clock())
            #print "the size of Setpoint is {}, Rpm {}, XData {}".format(len(SetpointData[i]),len(RpmData[i]),len(XData[i]))
            self.mplwidget.axes.plot(XData[i], SetpointData[i], colour[i]+"-", label="Setpoint {}".format(i))
            self.mplwidget.axes.plot(XData[i], RpmData[i], colour[i], label="Actual {}".format(i))
        self.mplwidget.axes.hold(False)
        self.mplwidget.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102),
                                   loc=3,
                                   ncol=MotorNum,
                                   mode="expand",
                                   borderaxespad=0.)
        self.mplwidget.axes.set_xlabel("Time [sec]")
        self.mplwidget.axes.set_ylabel("Speed [RPM]")
        self.mplwidget.draw()

app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
form = MainScreen()  # We set the form to be our ExampleApp (design)
form.show()  # Show the form
app.exec_()  # and execute the app

for i in range(len(motors)):
    if motors[i] is not None:
        motors[i].close()
        motorCallbacks[i] = None
        print "closed motor {}".format(i)

print "END OF PROGRAM"
sys.exit(0)

