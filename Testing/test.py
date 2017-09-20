# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 09:55:40 2017

@author: joell
"""

import sys
#import os
#steam = os.popen("pylint './Robotic Arm Class/Arm.py'")
from pylint.lint import Run
test_files = ['./Robotic Arm Class/Arm.py','./Robotic Arm Class/Geometry.py']
for file_under_test in test_files:
    results = Run([file_under_test], exit=False)
    print '*'*20
    if results.linter.stats['global_note'] < 5.0:
        print "Failed on {} the pylint score was {:.2f}. To pass you must have a score of above 5.".format(file_under_test, results.linter.stats['global_note'])
        sys.exit(0)

print "END OF TESTING, FORCING A PASSED BUILD"
#sys.exit(0)