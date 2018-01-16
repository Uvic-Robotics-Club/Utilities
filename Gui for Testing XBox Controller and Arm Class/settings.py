# -*- coding: utf-8 -*-
"""
Created on Tue Aug 01 11:17:48 2017

@author: joell

This class is just to have 1 class that contains all settings in a global dictionary called global dict.
This is to be imported to ALL classes to be used with the 'main program' gui
"""
import xbox_controller


def init():
    global globalDict
    global controller
    globalDict = {}
    globalDict['exit'] = False
    globalDict['network exit'] = False
    globalDict['connection'] = False
    globalDict['new image'] = False
    globalDict['img data'] = 0
    globalDict['ip'] = ""
    globalDict['received Comms'] = ""
    globalDict['active control'] = ['wheels','arm x,y,z']
    globalDict['message que'] = []
    globalDict['send que'] = []
    
    # globalDict['XinputJoystick'] is set when the xbox_controller is created
    globalDict['XinputJoystick'] = None
    controller = xbox_controller.XboxControls()
    
    globalDict['l1'] = 3
    globalDict['l2'] = 2
    globalDict['l3'] = 1.5
    globalDict['l0'] = 0.001
    globalDict['arm'] = ''
    globalDict['x_goal'] = 0.5
    globalDict['y_goal'] = 0.5
    globalDict['z_goal'] = 0.5
    
    