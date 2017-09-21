# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 12:02:35 2017

@author: joell
"""
from PyQt4 import QtCore, QtGui, uic  # Import the PyQt4 module we'll need
import sys  # We need sys so that we can pass argv to QApplication
from Arm import DisplayArm

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import radians,degrees

class MainScreen(QtGui.QMainWindow):
    def __init__(self):
        '''
        This is the main screen that the user interacts with.
        
        '''
        super(self.__class__, self).__init__()
        uic.loadUi('Main Arm Display.ui', self)
        
        #set up the repeating graph function on a timer
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(200) #update every [x]ms
        
        
        self.mplwidget.axes=Axes3D(self.mplwidget.figure)
        self.horizontalSlider.valueChanged.connect(self.SliderChanged)
        self.horizontalSlider_2.valueChanged.connect(self.SliderChanged)
        self.horizontalSlider_3.valueChanged.connect(self.SliderChanged)
        self.horizontalSlider_4.valueChanged.connect(self.SliderChanged)
        
        self.arm= DisplayArm([0.01,5,4,1.5],
                     units=0.0254, #this is inch to meter conversion
                     q1=45.0/360*2*np.pi,
                     q2=60.0/360*2*np.pi,
                     q3=-30.0/360*2*np.pi,
                     q4=-30.0/360*2*np.pi)
        
        [x,y,z] = self.arm.forward_kinematics()
        self.mplwidget.axes.plot(x,y,z)
        self.mplwidget.axes.scatter(x[0],y[0],z[0],color='g',label='p1')
        self.mplwidget.axes.scatter(x[1],y[1],z[1],color='r',label='p2')
        self.mplwidget.axes.scatter(x[2],y[2],z[2],color='m',label='p3')
        self.mplwidget.axes.scatter(x[3],y[3],z[3],color='g',label='p4')
        self.mplwidget.axes.legend()
        self.mplwidget.axes.plot([-.5,.5],[ 0,0],[0,0],color='k')
        self.mplwidget.axes.plot([0,0],[ -.5,.5],[0,0],color='k')
        
        self.mplwidget.axes.set_xlim((-10,10))
        self.mplwidget.axes.set_ylim((-10,10))
         
        
        self.mplwidget.axes.set_xlabel('x')
        self.mplwidget.axes.set_ylabel('y')
        self.mplwidget.axes.set_zlabel('z')

        
    def SliderChanged(self,horizontalSlider):
        '''
        This function is called every single time any one of the horizontal sliders is moved.
        It grabs the value of all of the sliders and updates the angles of joints of the arm
        '''
        
        self.arm.q1 = self.scale(self.horizontalSlider.value(),0,100,radians(-140),radians(140))
        self.arm.q2 = self.scale(self.horizontalSlider_2.value(),0,100,radians(0),radians(140))
        self.arm.q3 = self.scale(self.horizontalSlider_3.value(),0,100,radians(-145),radians(0))
        self.arm.q4 = -(self.arm.q2+self.arm.q3)
        
        #update the 'screens' to show the angle in degrees
        self.lcdNumber.display(degrees(self.arm.q1))
        self.lcdNumber_2.display(degrees(self.arm.q2))
        self.lcdNumber_3.display(degrees(self.arm.q3))
        self.lcdNumber_4.display(degrees(self.arm.q4))
    
    def scale(self,value,inMin,inMax,outMin,outMax):
        '''
        This function scales a number linearly
        This is the same as the arduino map class.
        
        Parameters
        ----------
        value : scalar
            the number to map
        
        inMin : scalar
            the lower bound of the value's current range
        
        inMax : scalar
        the upper bound of the value's current range
        
        outMin : scalar
            the lower bound of the value's target range
        
        outMax : scalar
            the upper bound of the value's target range
            
        Returns
        -------
        out : scalar
            The mapped value.
        '''
        value = float(value) #if value is not float you could end up with int math
        return (value-inMin)*(outMax-outMin)/(inMax-inMin)+outMin
        
    def updateFromGlobal(self):
        '''
        This function is called on a timer. This updates the main screen with values from rover.
        
        '''
        
        # find the joints for x, y, and z
        [x_joints,y_joints,z_joints] = self.arm.forward_kinematics()
        # clear and plot the data
        self.mplwidget.axes.clear()
        self.mplwidget.axes.plot3D(x_joints,y_joints,z_joints            ,color='b',label='links')
        self.mplwidget.axes.scatter3D(x_joints[0],y_joints[0],z_joints[0],color='g',label='p1')
        self.mplwidget.axes.scatter3D(x_joints[1],y_joints[1],z_joints[1],color='r',label='p2')
        self.mplwidget.axes.scatter3D(x_joints[2],y_joints[2],z_joints[2],color='m',label='p3')
        self.mplwidget.axes.scatter3D(x_joints[3],y_joints[3],z_joints[3],color='k',label='p4')
        
        self.mplwidget.axes.legend()
        self.mplwidget.axes.plot3D([-.5,.5],[ 0,0],[0,0],color='k')
        self.mplwidget.axes.plot3D([0,0],[ -.5,.5],[0,0],color='k')
        
        self.mplwidget.axes.set_xlabel('x')
        self.mplwidget.axes.set_ylabel('y')
        self.mplwidget.axes.set_zlabel('z')
        
        self.mplwidget.axes.set_xlim((-6,6))
        self.mplwidget.axes.set_ylim((-6,6))
        self.mplwidget.axes.set_zlim((-5,5))
        

        self.mplwidget.axes.grid(True)
        self.mplwidget.axes.figure.canvas.draw()
    
    def setSliderValues(self,sliderNumber,value):
        '''
        This sets the value of the slider. This is the main way to interact with the outside world.
        
        Parameters
        ----------
        sliderNumber : integer
            slider 1 is the rotation of the base
            slider 2 is the rotation of the first link
            slider 3 is the rotation of the third link
            slider 4 is the grabber open and close
        
        value : integer
            percentage of the slider from 0-100
        
        Returns
        -------
        None
        '''
        if(sliderNumber==1):
            self.horizontalSlider.setValue(int(value))
        elif(sliderNumber==2):
            self.horizontalSlider_2.setValue(int(value))
        elif(sliderNumber==3):
            self.horizontalSlider_3.setValue(int(value))
        elif(sliderNumber==4):
            self.horizontalSlider_4.setValue(int(value))


app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
form = MainScreen()  # We set the form to be our ExampleApp (design)
form.show()  # Show the form
app.exec_()  # and execute the app