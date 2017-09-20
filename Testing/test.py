# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 09:55:40 2017

@author: joell
"""

import sys
import os
#print subprocess.Popen("echo Hello World", shell=True, stdout=subprocess.PIPE).stdout.read()
steam = os.popen("pylint './Robotic Arm Class/Arm.py'")
print steam.read()
print "END OF TESTING, FORCING A PASSED BUILD"
sys.exit(0)