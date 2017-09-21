# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 09:55:40 2017

@author: joell
"""

import sys
from pylint.lint import Run

def robotic_arm_tests():
	'''
	Tests a few files in the robotic arm folder.

	Parameters
	----------
	None

	Returns
	-------
	None
	'''
	folder = './Robotic Arm Class/'
	test_files = ['Arm.py', 'Geometry.py', 'inverseKinematicUtilities.py','FABRIK demo.py']
	for file_under_test in test_files:
		results = Run([folder+file_under_test], exit=False)
		print '*'*20
		if results.linter.stats['global_note'] < 5.0:
			print """
			Failed on {} the pylint score was {:.2f}.
			To pass you must have a score of above 5.
			""".format(folder+file_under_test, results.linter.stats['global_note'])
			sys.exit(1)

robotic_arm_tests()
print "END OF TESTING, THIS IS A PASSED BUILD"
sys.exit(0)
