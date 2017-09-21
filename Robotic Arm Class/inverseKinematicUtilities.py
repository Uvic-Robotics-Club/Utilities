# -*- coding: utf-8 -*-
"""
Created on Sat Sep 02 12:16:00 2017

@author: joell
"""

import numpy as np
# pylint: disable=C0103
def angle_from_dot_product(a, b):
	'''
	Parameters
	----------
	a, b : array_like
		1x3 arrays that you want to find the angle betwen

	Returns
	-------
	theta : scalar
		the angle from the dot product of a and b
	'''
	a_mag = np.sqrt(np.power(a[0], 2)+np.power(a[1], 2)+np.power(a[2], 2))
	b_mag = np.sqrt(np.power(b[0], 2)+np.power(b[1], 2)+np.power(b[2], 2))

	theta = np.arccos(np.power(a_mag*b_mag, -1)*(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]))
	return theta


def dh(theta, d, a, alpha):
	"""
	Perameters
	----------
	theta : scalar
		rotation around the z axis
	d : scalar
		translation in the z axis
	a : scalar
		translation in the x axis
	alpha : scalar
		rotation around the x axis


	Returns
	-------
	out : array_like
		Tranformation maxtrix of how to acheive the desired translation and rotation. This array is 4x4


	More Information
	----------------
	More information can be found here https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
	"""
	return np.array([[np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
					 [np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha)*np.sin(theta), a*np.sin(theta)],
					 [0, np.sin(alpha), np.cos(alpha), d],
					 [0, 0, 0, 1.0]])


def project_along_vector(x1, y1, z1, x2, y2, z2, L):
	'''
	 Solve for the point px, py, pz, that is
	 a vector with magnitude L away in the direction between point 2 and point 1,
	 starting at point 1

	 Parameters
	 ----------
	 x1, y1, z1 : scalars
		 point 1
	 x2, y2, z2 : scalars
		 point 2
	 L : scalar
		 Magnitude of vector?

	Returns
	-------
	out : array_like
		projected point on the vector

	'''

	# vector from point 1 to point 2
	vx = x2-x1
	vy = y2-y1
	vz = z2-z1
	v = np.sqrt(np.power(vx, 2)+np.power(vy, 2)+np.power(vz, 2))

	ux = vx/v
	uy = vy/v
	uz = vz/v

	# Need to always project along radius
	# Project backwards
	px = x1+L*ux
	py = y1+L*uy
	pz = z1+L*uz

	return np.array([px, py, pz])


def inverseKinematics_fabrik(l1, l2, l3, l4, x_joints, y_joints, z_joints, x, y, z, tol_limit, max_iterations):
	'''
	Perameters
	----------

	q1o, q2o, q3o, q4o : array_like
		are the initial positions of each joint.
	x, y, z : scalars
		desired location
	tol_limit : scalar
		tolerance between the desired location and actual one after iteration
	max_iterations : scalar
		maximum itterations that the function is allowed to run


	Returns
	-------
	q1 : scalar
		rotation of the base about the z axis
	q2 : scalar
		rotation of first link relative to the base
	q3 : scalar
		rotation of the second link relative to the first link
	q4 : scalar
		rotation of the third link relative to the second link
	p_joints : array_like
		x, y, z points for all the links in the format of p_joints =  [[p1x, p2x, p3x, p4x], [p1y, p2y, p3y, p4y], [p1z, p2z, p3z, p4z]]
	itterations : scalar
		how many itterations it took


	More Information
	----------------
	Overall algorithim: solve for unit vectors going backwards from the desired point to the original point of the joints, then forwards from the origin, see youtube video https://www.youtube.com/watch?v = UNoX65PRehA&t = 817s

	'''

	# Find base rotation

	# returns -pi to pi, different from arctan2
	q1_o = np.arctan2(y_joints[-1], x_joints[-1])	   # Initial angle of where end effector is
	q1 = np.arctan2(y, x)							   # Desired angle

	base_rotation = q1-q1_o					   # Base rotation




	# Base rotation matrix about z
	R_z = np.array([[np.cos(base_rotation), -np.sin(base_rotation), 0.0],
		[np.sin(base_rotation), np.cos(base_rotation), 0.0],
		[0.0, 0.0	, 1.0]])

	# Rotate the location of each joint by the base rotation
	# This will force the FABRIK algorithim to only solve
	# in two dimensions, else each joint will move as if it has
	# a 3 DOF range of motion
	#print 'inside the fabrik method and x_joints is'
	#print x_joints
	p4 = np.dot(R_z, [x_joints[3], y_joints[3], z_joints[3]])
	p3 = np.dot(R_z, [x_joints[2], y_joints[2], z_joints[2]])
	p2 = np.dot(R_z, [x_joints[1], y_joints[1], z_joints[1]])
	p1 = np.dot(R_z, [x_joints[0], y_joints[0], z_joints[0]])

	# Store the (x, y, z) position of each joint
	p4x = p4[0]
	p4y = p4[1]
	p4z = p4[2]

	p3x = p3[0]
	p3y = p3[1]
	p3z = p3[2]

	p2x = p2[0]
	p2y = p2[1]
	p2z = p2[2]

	p1x = p1[0]
	p1y = p1[1]
	p1z = p1[2]

 # store starting point of the first joint
	p1x_o = p1x
	p1y_o = p1y
	p1z_o = p1z

	iterations = 0
	for q in range(1, max_iterations+1):#for q = 1:max_iterations
		# Make sure the desired x, y, z point is reachable
		if np.sqrt(np.power(x, 2)+np.power(y, 2)+np.power(z, 2)) > (l2+l3+l4):
			print ' desired point is likely out of reach'


		# Overall algorithim: solve for unit vectors going backwards from the
		# desired point to the original point of the joints, then forwards from the
		# origin, see youtube video https://www.youtube.com/watch?v = UNoX65PRehA&t = 817s


		# backwards
		#project_along_vector(x1, y1, z1, x2, y2, z2, L)

		[p3x, p3y, p3z] = project_along_vector(x, y, z, p3x, p3y, p3z, l4)
		[p2x, p2y, p2z] = project_along_vector(p3x, p3y, p3z, p2x, p2y, p2z, l3)
		[p1x, p1y, p1z] = project_along_vector(p2x, p2y, p2z, p1x, p1y, p1z, l2)

		# forwards

		[p2x, p2y, p2z] = project_along_vector(p1x_o, p1y_o, p1z_o, p2x, p2y, p2z, l2)
		[p3x, p3y, p3z] = project_along_vector(p2x, p2y, p2z, p3x, p3y, p3z, l3)
		[p4x, p4y, p4z] = project_along_vector(p3x, p3y, p3z, x, y, z, l4)

		# Solve for tolerance between iterated point and desired x, y, z,
		tolx = p4x-x
		toly = p4y-y
		tolz = p4z-z

		# Make tolerance relative to x, y, z
		tol = np.sqrt(np.power(tolx, 2)+np.power(toly, 2)+np.power(tolz, 2))

		iterations = iterations+1

		# Check if tolerance is within the specefied limit
		if tol<tol_limit:
			break

	# Re-organize points into a big matrix for plotting elsewhere
	p_joints =  np.array([[p1x, p2x, p3x, p4x],
						[p1y, p2y, p3y, p4y],
						[p1z, p2z, p3z, p4z]])
	#print "p4 is {:.2f}, {:.2f}, {:.2f}\npx is {:.2f}, {:.2f}, {:.2f}".format(p4x, p4y, p4z, x, y, z)

	# Return the joint angles by finding the angles with the dot produvt
	v21 = np.array([p2x-p1x, p2y-p1y, p2z-p1z])
	v32 = np.array([p3x-p2x, p3y-p2y, p3z-p2z])
	v43 = np.array([p4x-p3x, p4y-p3y, p4z-p3z])

	# returns -pi to pi
	q2 = np.arctan2((p2z-p1z), np.sqrt(np.power(p2x-p1x, 2)+np.power(p2y-p1y, 2)))

	# Negative sign because of dh notation, a rotation away from the previous link
	# and towards the x-y plane is a negative moment about the relative z axis.
	# the relative z axis of each link is out of the page if looking at the arm
	# in 2D
	# the x axis in dh convention is typically along the link direction.

	q3 = -1*angle_from_dot_product(v21, v32)
	q4 = -1*angle_from_dot_product(v32, v43)

	return [q1, q2, q3, q4, p_joints, iterations]


