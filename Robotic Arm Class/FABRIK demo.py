# -*- coding: utf-8 -*-
"""
Created on Sat Sep 02 11:21:39 2017

@author: joell
"""
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.animation as animation

from math import pi

from inverseKinematicUtilities import dh, inverseKinematics_fabrik

meters = 1.0
feet = 0.3048
#pylint: disable=C0103
# Initial angles
# all angles are taken within the plane that is parallel to all the links

q1 = 45.0/360*2*pi   # rotation of the base about the z axis
q2 = 60.0/360*2*pi   # rotation of first link relative to the base
q3 = -30.0/360*2*pi   # rotation of the second link relative to the first link
q4 = -30.0/360*2*pi   # rotation of the third link relative to the second link

l1 = .02	  # z-axis distance from the global xyz frame to the first joint
		  # this code assumes the global frame of
		  # reference has its z-axis centered with the
		  # first joint

l2 = 3.0*feet	 # length of the first link
l3 = 2.0*feet	 # length of the second link
l4 = 1.5*feet	 # length of the third link


T10 = dh(q1, l1, 0, pi/2.0) # Create transformation matrix from joint 1 to 0
T21 = dh(q2, 0, l2, 0)   # Create transformation matrix from joint 2 to 1
T32 = dh(q3, 0, l3, 0)   # Create transformation matrix from joint 3 to 2
T43 = dh(q4, 0, l4, 0)   # Create transformation matrix from joint 4 to 3

#NOTE: THESE ARE DOT PRODUCTS
T20 = np.dot(T10, T21)	  # transformation matrix from frame 2 to global 0
T30 = np.dot(T20, T32)	  # transformation matrix from frame 2 to global 0
T40 = np.dot(T30, T43)	  # transformation matrix from frame 2 to global 0


# Initial position of each joint
#was accessing each element with x[1:3, 4] -> translated that into X[0:4, 3]
p_joints = [T10[0:3, 3], T20[0:3, 3], T30[0:3, 3], T40[0:3, 3]]

p_joints = np.transpose(p_joints)

x_joints_o = p_joints[0]
y_joints_o = p_joints[1]
z_joints_o = p_joints[2]

x_joints = p_joints[0]
y_joints = p_joints[1]
z_joints = p_joints[2]


# Time matrix for example helix path
N_cycles = 3.0
T = 1.0

dt = T/100.0
Nt = int(round(N_cycles*T/dt))
#orignal was t = [0:1:Nt-1]*dt -> translated to range(0, (Nt-1)*dt, dt)
t = np.array(range(0, (Nt-1), 1))*dt

x = 1.7*np.cos(2*pi*t+pi/8)
y = 1.7*np.sin(2*pi*t+pi/8)
z = -.05-.05*t

tol_limit = .005



# ROBOT FIGURE
f_2 = plt.figure(2)
ax = f_2.add_subplot(111, projection = '3d')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()

def animate(n):
	global x_joints, y_joints, z_joints

	# Call Inverse Kinematics function to solve for joint angles
	[q1, q2, q3, q4, _, _] = inverseKinematics_fabrik(l1, l2, l3, l4,
		x_joints, y_joints, z_joints,
		x[n], y[n], z[n], tol_limit, 100)

	# Use forward kinmatics to move robot
	# Initial forward kinematics

	T10 = dh(q1, l1, 0, pi/2)  # Create transformation matrix from 0 to 1
	T21 = dh(q2, 0, l2, 0)   # Create transformation matrix from 2 to 1
	T32 = dh(q3, 0, l3, 0)   # Create transformation matrix from 3 to 2
	T43 = dh(q4, 0, l4, 0)   # keep q4 constant

	T20 = np.dot(T10, T21)
	T30 = np.dot(T20, T32)
	T40 = np.dot(T30, T43)  # Transformation matrix from end effector to the global frame

	p_joints = [T10[0:3, 3], T20[0:3, 3], T30[0:3, 3], T40[0:3, 3]]

	p_joints = np.transpose(p_joints)

	x_joints_o = p_joints[0]
	y_joints_o = p_joints[1]
	z_joints_o = p_joints[2]

	x_joints = p_joints[0]
	y_joints = p_joints[1]
	z_joints = p_joints[2]

	ax.clear()
	plt.hold(True)
	ax.plot3D(x_joints, y_joints, z_joints)
	ax.scatter3D(x_joints_o[0], y_joints_o[0], z_joints_o[0], color='g', label='p1')
	#plt.hold(True)
	ax.scatter3D(x_joints_o[1], y_joints_o[1], z_joints_o[1], color='r', label='p2')
	#plt.hold(True)
	ax.scatter3D(x_joints_o[2], y_joints_o[2], z_joints_o[2], color='m', label='p3')
	#plt.hold(True)
	ax.scatter3D(x_joints_o[3], y_joints_o[3], z_joints_o[3], color='black', label='p4')
	#plt.hold(True)
	ax.legend()
	#plt.hold(True)
	ax.plot3D(x, y, z)
	#plt.hold(True)
	ax.scatter3D(0, 0, 0)
	#plt.hold(True)
	ax.plot3D([-.5, .5], [0, 0], [0, 0], color='black')
	#plt.hold(True)
	ax.plot3D([0, 0], [-.5, .5], [0, 0], color='black')
	#plt.hold(True)

	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	#view([62 16])
	ax.grid(True)
	plt.hold(False)
	plt.show()

ani = animation.FuncAnimation(f_2, animate, range(1, Nt-1), interval = 10)
