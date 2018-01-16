# -*- coding: utf-8 -*-
"""
Created on Fri Jan 05 20:40:22 2018

@author: joell
"""

class Callbacks:
    def __init__(self, localArduino=None):
        self.ardu = localArduino

    def Setpoint(self, value):
        try:
            self.ardu.write("s{}".format(value))
            self.ardu.flush()
            print "Changing the setpoint to {}".format(value)
            sys.stdout.flush()
        except Exception as e:
            print "Setpoint CALLBACK ERROR: "
            print e.message
            sys.stdout.flush()

    def Kp(self,value):
        try:
            self.ardu.write("p{}".format(value))
            self.ardu.flush()
            print "sending Kp value of {}".format(value)
            sys.stdout.flush()
        except Exception as e:
            print "Kp CALLBACK ERROR: "
            print e.message
            sys.stdout.flush()

    def Kd(self,value):
        try:
            self.ardu.write("d{}".format(value))
            self.ardu.flush()
            print "sending Kd value of {}".format(value)
            sys.stdout.flush()
        except Exception as e:
            print "Kd CALLBACK ERROR: "
            print e.message
            sys.stdout.flush()

    def Ki(self,value):
        try:
            self.ardu.write("i{}".format(value))
            self.ardu.flush()
            print "sending Ki value of {}".format(value)
            sys.stdout.flush()
        except Exception as e:
            print "Ki CALLBACK ERROR: "
            print e.message
            sys.stdout.flush()