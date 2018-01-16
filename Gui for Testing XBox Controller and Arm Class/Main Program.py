# -*- coding: utf-8 -*-
"""
Created on Tue May 23 09:50:13 2017

@author: joell
"""

'''
Import in this order:
PyQt4 so that we can have a interface and work with it
sys so we can pass arguments to the interface
numpy so we can do matrix math properly
time so we can create some time stuff
threading so we can have multipul things running at the same time (asyncronous)
json because all sending is done trough dictionaries
cv2 to display and to do some other stuff with images

colored to make the terminal different colors
arm to do the inverse kinematics and stuff like that

'''

from PyQt4 import QtCore, QtGui, uic  # Import the PyQt4 module we'll need
import sys  # We need sys so that we can pass argv to QApplication
import numpy as np
import time
import socket
import threading
import json
import cv2

try:
    from termcolor import colored
except ImportError:
    print "Please Instal the termcolor program. It can be installed using 'pip install termcolor'"
    sys.exit("Exiting because of failure to have termcolor installed.")

from mpl_toolkits.mplot3d import Axes3D

#adjust this to include the utility class for the robotic arm class for the indvidual computer
sys.path.append("D:\Documents\GitHub\Utility")
sys.path.append("D:\Documents\GitHub\Utility\Robotic Arm Class")
try:
    from Arm import DisplayArm
except ImportError:
    print colored("I could not import some of the Utilities that I need, did you make sure to include the github utility class to your python path?",'red')
    sys.exit("I didnt import my utilities class! RUNNING FOR SAFETY! EXITING")


import settings
import xbox_controller

from math import radians
import urllib2


def NetworkStarter():
    '''
    This is a helper function. This function start 2 threads for listend for pictures and sending commands

    Parameters
    ----------
    None

    Returns
    -------
    None
    '''
    sys.stdout.write("setting up the hosts and ports\n")
    HOST = socket.gethostbyname(socket.gethostname())
    SENDPORT = 314
    print "host is " + HOST + " and sending on port %d"%(SENDPORT)
    send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    send.bind((HOST, SENDPORT))
    t2 = threading.Thread(target=NetworkSender,kwargs={'s':send})
    #then start a new thread for network sender
    LISTENPORT = 314*2
    print "host is " + HOST + " and listening on port  %d"%(LISTENPORT)
    listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listen.bind((HOST, LISTENPORT))
    #then start a new thread for network sender
    t4 = threading.Thread(target=NetworkListener,kwargs={'s':listen})
    t2.start()
    #t4.start()



def recvall(sock, count):
    '''
    Helper Function for receiving data.

    Parameters
    ----------
    sock : socket
        this is an open socket that can be read from
    count : integer
        Number of bytes to receive

    Returns
    -------
    buf : string
        returns a string representation of what was received
    '''
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf


def NetworkSender(s):
    '''
    This function sends commands to the rover. It sends commands via json.dumps so commands must be in the form of a dictionary.
    This function acts as an infinte loop to constantly send data. It should not end unless the program has to be restarted.

    Parameters
    ----------
    s : socket
        This is a socket that is already binded to a socket but has not accepted a client yet

    Returns
    -------
    None


    '''
    while True:
        s.listen(1)
        conn, addr = s.accept()
        sys.stdout.write( 'Connected by'+ str(addr)+"\n")
        sys.stdout.flush()
        settings.globalDict['connection']=True
        settings.globalDict['ip'] = addr
        while True:
            try:
                temp = json.dumps(settings.controller.getMotors())
                conn.send(temp)
                time.sleep(0.01)

                if(len(settings.globalDict['send que']) != 0):
                    while len(settings.globalDict['send que']) != 0:
                        conn.send(json.dumps(settings.globalDict['send que'].pop(0)))
                        time.sleep(0.01)

            except Exception as e:
                sys.stderr.write( "cought an error sending data. The error is: " + e.message + "\n")
                sys.stderr.flush()
                break
            if(settings.globalDict['exit'] or settings.globalDict['network exit']):
                conn.send(json.dumps({'exit':0}))
                conn.close()
                sys.stdout.write( "ending the sending thread\n")
                sys.stdout.flush()
                return

        conn.close()
        settings.globalDict['connection']=False
        settings.globalDict['ip'] = ""


def NetworkListener(s):
    '''
    This function listens to data being sent back from the rover.
    The way it is set up right now it can receive pictures from the rover and update the rest of the program.
    In the future the rover will send back more sensor data and this will be the place where it is read and distributed.

    Parameters
    ----------
    s : socket
        This is a socket that is already binded to a socket but has not accepted a client yet

    Returns
    -------
    None

    '''
    while True:
        s.listen(1)
        conn, addr = s.accept()
        sys.stdout.write( 'Connected by'+ str(addr)+"\n")
        sys.stdout.flush()
        settings.globalDict['S connection']=True
        settings.globalDict['S ip'] = addr


        while True:
            try:
                string = conn.recv(1024)

                if("new frame" in string):
                    #print colored('Incoming new image!','magenta')
                    data=""
                    length = recvall(conn,16)
                    stringData = recvall(conn, int(length))
                    data = np.fromstring(stringData, dtype='uint8')
                    decimg = cv2.imdecode(data,1)
                    decimg = cv2.cvtColor(decimg, cv2.COLOR_BGR2RGB)
                    settings.globalDict['new image'] = True
                    settings.globalDict['img data'] = decimg
                    #cv2.imwrite('latestImage.jpg',decimg)

                if(string != ''):
                    print colored(string,'green')


                time.sleep(0.01)
            except Exception as e:
                sys.stderr.write( "cought an error sending data. The error is: " + e.message + "\n")
                sys.stderr.flush()
                break
            if(settings.globalDict['exit'] or settings.globalDict['network exit']):
                conn.close()
                sys.stdout.write( "ending the receiving thread\n")
                sys.stdout.flush()
                return
        conn.close()
        settings.globalDict['connection']=False
        settings.globalDict['ip'] = ""








def button_callback_1(button,value):
    '''
    This is a callback function that is activated when the right bumber is pressed.
    It should change what the active control element is.

    Current control elements are:
    * rover direction
    * x, y, z for robotic arm

    Parameters
    ----------
    button : integer
        this is the button that is being pressed on the xbox controller
    value : integer
        this is a 1 or a 0 depending on if the button is pressed down or not

    Returns
    -------
    None

    '''
    global form
    settings.globalDict['active control'] = np.roll(settings.globalDict['active control'],1)
    settings.globalDict['message que'].append("current control is now set to {0}".format(settings.globalDict['active control'][0]))

def axis_callback(axis,value):
    '''
    This is a small function that is called every time one of the joysticks or triggers is moved on an xbox controller

    '''
    #print 'this is the %s axis callback'%(str(axis))
    settings.globalDict['y_goal'] = value*10.0
    #settings.globalDict['message que'].append("current control is now set to {0}".format(settings.globalDict['active control'][0]))
    #settings.controller.values[axis] = value

def axis_callback1(axis,value):
    settings.globalDict['x_goal'] = value*10.0

def axis_callback2(axis,value):
    settings.globalDict['z_goal'] = value*10.0






class ControlsDialog(QtGui.QDialog):
    '''
    This class is just for the controls dialog panel that allows you to change what buttons do what.

    '''
    def __init__(self):
        super(self.__class__, self).__init__()
        uic.loadUi('Controls.ui', self)
        self.label_2.setPixmap(QtGui.QPixmap("360_controller.png"))

        # set the items for each comboBox
        self.l_trigger_combo.addItems(settings.controller.validOptions)
        self.r_trigger_combo.addItems(settings.controller.validOptions)
        self.l_thumb_x_combo.addItems(settings.controller.validOptions)
        self.l_thumb_y_combo.addItems(settings.controller.validOptions)
        self.steering_style_combo.addItems(settings.controller.profiles)
        self.speed_style_combo.addItems(settings.controller.profiles)

        # set the combo boxes to the right indexes
        self.l_trigger_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['left_trigger']))
        self.r_trigger_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['right_trigger']))
        self.l_thumb_x_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['l_thumb_x']))
        self.l_thumb_y_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['l_thumb_y']))


        #deal with the save, open and cancel buttons
        self.buttonBox.button(QtGui.QDialogButtonBox.Save).clicked.connect(self.saveButton)
        self.buttonBox.button(QtGui.QDialogButtonBox.Open).clicked.connect(self.openButton)
        self.buttonBox.button(QtGui.QDialogButtonBox.Cancel).clicked.connect(self.cancelButton)
        self.buttonBox.button(QtGui.QDialogButtonBox.Apply).clicked.connect(self.applyButton)




    def saveInfo(self,save = True):
        '''
        This function "saves" the current control scheme so that it can be used at another time.

        Parameters
        ----------
        save : boolean
            If this is true it brings up a file chooser so you can save the controls to a file. If false it only applies the settings without saving them.

        Returns
        -------
        None

        '''
        if(save):
            fileName = QtGui.QFileDialog.getSaveFileName(self,'save config','','Configs (*.xbox)')
            if(fileName != None and fileName != ''):
                settings.controller.setControl('left_trigger',settings.controller.validOptions[self.l_trigger_combo.currentIndex()])
                settings.controller.setControl('right_trigger',settings.controller.validOptions[self.r_trigger_combo.currentIndex()])
                settings.controller.setControl('l_thumb_x',settings.controller.validOptions[self.l_thumb_x_combo.currentIndex()])
                settings.controller.setControl('l_thumb_y',settings.controller.validOptions[self.l_thumb_y_combo.currentIndex()])
                settings.controller.save(fileName)
                self.close()
        else:
            settings.controller.setControl('left_trigger',settings.controller.validOptions[self.l_trigger_combo.currentIndex()])
            settings.controller.setControl('right_trigger',settings.controller.validOptions[self.r_trigger_combo.currentIndex()])
            settings.controller.setControl('l_thumb_x',settings.controller.validOptions[self.l_thumb_x_combo.currentIndex()])
            settings.controller.setControl('l_thumb_y',settings.controller.validOptions[self.l_thumb_y_combo.currentIndex()])
            self.close()

    def saveButton(self):
        '''
        Callback for the save button on the Controls Dialog. This calls the save info function making sure to save the information to a file.

        Parameters
        ----------
        None

        Returns
        -------
        None

        '''
        self.saveInfo(save=True)

    def applyButton(self):
        '''
        Callback for the apply button on the Controls Dialog. This calls the save info function making sure to NOT to save the information to a file.

        Parameters
        ----------
        None

        Returns
        -------
        None

        '''
        self.saveInfo(save=False)


    def openButton(self):
        '''
        Callback for the open button on the Controls Dialog. This function opens a file chooser and lets the user open a .xbox file that they have saved from before.
        Once the file has been loaded the settings are applied right away.

        Parameters
        ----------
        None

        Returns
        -------
        None

        '''
        fileName = QtGui.QFileDialog.getOpenFileName(self,'open configuration','','Configs (*.xbox);;All (*.*)')
        if(fileName != None and fileName != ''):
            settings.controller.load(fileName)

            # set the combo boxes to the right indexes
            self.l_trigger_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['left_trigger']))
            self.r_trigger_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['right_trigger']))
            self.l_thumb_x_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['l_thumb_x']))
            self.l_thumb_y_combo.setCurrentIndex(settings.controller.validOptions.index(settings.controller.controls['l_thumb_y']))


    def cancelButton(self):
        '''
        Callback for the cancel button on the Controls Dialog. This closes the dialog without appling, saving, or opening any other files.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''

        self.close()




class MainScreen(QtGui.QMainWindow):
    def __init__(self):
        '''
        This is the main screen that the user interacts with.

        '''
        super(self.__class__, self).__init__()
        uic.loadUi('Main Window.ui', self)

        self.startNetworkThread.clicked.connect(self.internalNetworkStarter)
        self.action_Controls.triggered.connect(self.changeControlsMenu)

        self.qualityComboBox.currentIndexChanged.connect(self.comboBoxChanged)
        self.qualitySlider.valueChanged.connect(self.qualitySliderChanged)


        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(100)

        settings.globalDict['starting threads'][0].start()


        self.mplwidget.axes=Axes3D(self.mplwidget.figure)

        settings.globalDict['arm']= DisplayArm([settings.globalDict['l0'],settings.globalDict['l1'],settings.globalDict['l2'],settings.globalDict['l3']],
                     units=0.3048,
                     q1=45.0/360*2*np.pi,
                     q2=60.0/360*2*np.pi,
                     q3=-30.0/360*2*np.pi,
                     q4=-30.0/360*2*np.pi)

        [x,y,z] = settings.globalDict['arm'].forward_kinematics()
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

        settings.globalDict['XinputJoystick'].button_attach_callback(10,button_callback_1)
        settings.globalDict['XinputJoystick'].button_attach_callback(5,self.__internalClose__)
        settings.globalDict['XinputJoystick'].axis_attach_callback('r_thumb_x',axis_callback)
        settings.globalDict['XinputJoystick'].axis_attach_callback('r_thumb_y',axis_callback1)
        settings.globalDict['XinputJoystick'].axis_attach_callback('l_thumb_y',axis_callback2)



        #self.pushButton_1.clicked.connect(self.pushButton1func)
        #self.pushButton_2.clicked.connect(self.pushButton2func)
        #self.comboBox.activated.connect(self.comboBoxAction)
        #self.dial.valueChanged.connect(self.dialChanging)
        #self.actionSomething.triggered.connect(self.fileThing)
    def __internalClose__(self,button,value):
        '''
        this is an internal only method that closes the main window
        '''
        self.close()


    def internalNetworkStarter(self):
        '''
        This starts and stops the Networkstarter function. This function is triggered by the 'start network threads' button on the main screen.
        '''
        if "Start" in self.startNetworkThread.text():
            settings.globalDict['network exit'] = False
            t1 = threading.Thread(target=NetworkStarter)
            t1.start()
            #settings.globalDict['starting threads'][1].start()
            self.startNetworkThread.setText('Stop Network Threads')
        else:
            settings.globalDict['network exit'] = True
            self.startNetworkThread.setText('Start Network Threads')

    def comboBoxChanged(self,n):
        '''
        This function is called every time the resolution combo box changes.

        TODO: Add the ability to let the user know that they cant change anything if the rover is not connected.

        '''
        n = self.qualityComboBox.currentText()
        if('1080' in n):
            settings.globalDict['send que'].append({'width':1920})
            settings.globalDict['send que'].append({'height':1080})
        elif('720' in n):
            settings.globalDict['send que'].append({'width':1280})
            settings.globalDict['send que'].append({'height':720})
        elif('480' in n):
            settings.globalDict['send que'].append({'width':854})
            settings.globalDict['send que'].append({'height':480})
        elif('360' in n):
            settings.globalDict['send que'].append({'width':640})
            settings.globalDict['send que'].append({'height':360})
        elif('240' in n):
            settings.globalDict['send que'].append({'width':426})
            settings.globalDict['send que'].append({'height':240})

    def qualitySliderChanged(self,n):
        '''
        This function is called every time the quality slider is changed.

        TODO: Add the ability to let the user know that they cant change anything if the rover is not connected.
        '''
        settings.globalDict['send que'].append({'quality':n})

    def changeControlsMenu(self):
        '''
        This opens up the controls dialog box. This is activated from the menu bar
        '''
        self.dialog = ControlsDialog()
        #this will create a modal dialog
        #self.dialog.exec_()
        #this will create a modeless dialog
        self.dialog.show()

    def updateFromGlobal(self):
        '''
        This function is called on a timer. This updates the main screen with values from rover.

        Currently implemented are:

        #. sending motor speeds
        #. displaying camera feed from rover
        #. showing messages from other parts of the program

        '''
        self.connected_value.setText(str(settings.globalDict['connection']))
        self.IP_value.setText(str(settings.globalDict['ip']))
        #self.IP_value.setText(str(controller.values['left_trigger']*100))
        M = settings.controller.getMotors()
        #M = settings.controller.M

        if(M!= None):
            self.top_left_dial.setValue(M["M2"])
            self.top_right_dial.setValue(M["M4"])
            self.bot_left_dial.setValue(M["M1"])
            self.bot_right_dial.setValue(M["M3"])

        if(settings.globalDict['new image']):
            height, width, channel = settings.globalDict['img data'].shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(settings.globalDict['img data'].data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.ImageLabel.setPixmap(QtGui.QPixmap(qImg))
            #self.ImageLabel.setPixmap(qpm)
            settings.globalDict['new image']=False

        if(len(settings.globalDict['message que']) != 0):
            self.statusbar.showMessage(settings.globalDict['message que'].pop(0),3000)

        # find angles for x, y, and z
        [q1,q2,q3,q4,x_joints,y_joints,z_joints] = settings.globalDict['arm'].inverse_kinematics([settings.globalDict['x_goal'],settings.globalDict['y_goal'],settings.globalDict['z_goal']],0.001,1000)

        # clear and plot the data
        self.mplwidget.axes.clear()
        #plt.hold(True)
        self.mplwidget.axes.plot3D(x_joints,y_joints,z_joints            ,color='b',label='links')
        #self.mplwidget.axes.scatter3D(settings.globalDict['x_goal'],settings.globalDict['y_goal'],settings.globalDict['z_goal'],color='y',label='path')
        self.mplwidget.axes.scatter3D(x_joints[0],y_joints[0],z_joints[0],color='g',label='p1')
        self.mplwidget.axes.scatter3D(x_joints[1],y_joints[1],z_joints[1],color='r',label='p2')
        self.mplwidget.axes.scatter3D(x_joints[2],y_joints[2],z_joints[2],color='m',label='p3')
        self.mplwidget.axes.scatter3D(x_joints[3],y_joints[3],z_joints[3],color='k',label='p4')
        #ax.scatter3D(0,0,0                              ,color='k')
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




def startGUI():
    '''
    This starts the Main screen in the main loop. When you try and start a Qt application from a thread it complains.
    This function is blocking.
    '''
    global form
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = MainScreen()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
    settings.globalDict['exit'] = True
    return

if __name__ == '__main__':  # if we're running file directly and not importing it
    settings.init()

    threads = []
    t1 = threading.Thread(target=xbox_controller.joystickWorker)
    t2 = threading.Thread(target=NetworkStarter)

    settings.globalDict['starting threads'] = [t1,t2]
    startGUI()