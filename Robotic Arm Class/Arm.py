# -*- coding: utf-8 -*-
"""
Created on Tue Aug 01 14:56:44 2017

@author: joell

"""

import numpy as np
from Geometry import Point, Vector

class DisplayArm(object):
	"""
	A class for a 3 link robotic arm with forward and inverse kinematics

	Parameters
	----------
	lengths : array_like
		the length of each link in the form [offset, link1, link2, link3]
	units : scalar (optional)
		conversion factor between whatever units the lengths are and meters.
		For example, if the lengths were given in feet then
		the conversion factor would be 0.3048
	q1 : scalar (optional)
		inital rotation of the base in radians
	q2 : scalar (optional)
		initial rotation of link 1 from the base in radians
	q3 : scalar (optional)
		intial rotation of link 2 from link 1 in radians
	q4 : scalar (optional)
		initial rotation of link 3 from link 2 in radians

	Example
	-------
	>>> arm = DisplayArm([0.02,3,2,1.5],
				units=0.3048,
				q1=45.0/360*2*np.pi,
				q2=60.0/360*2*np.pi,
				q3=-30.0/360*2*np.pi,
				q4=-30.0/360*2*np.pi)
	"""
	# pylint: disable=too-many-instance-attributes
	def __init__(self, lengths, *args, **kwargs):
		units = 1.0		# assume meters
		self.q1 = 0.0	# rotation of the base
		self.q2 = 0.0	# rotation of link 1 from the base
		self.q3 = 0.0	# rotation of link 2 from link 1
		self.q4 = 0.0	# rotation of link 3 from link 2

		if len(args) >= 1:
			units = args[0]
		if len(args) >= 2:
			self.q1 = args[1]
		if len(args) >= 3:
			self.q2 = args[2]
		if len(args) >= 4:
			self.q3 = args[3]
		if len(args) >= 5:
			self.q4 = args[4]

		if kwargs.has_key("units"):
			self.units = kwargs["units"]
		if kwargs.has_key("q1"):
			self.q1 = kwargs["q1"]
		if kwargs.has_key("q2"):
			self.q2 = kwargs["q2"]
		if kwargs.has_key("q3"):
			self.q3 = kwargs["q3"]
		if  kwargs.has_key("q4"):
			self.q4 = kwargs["q4"]

		self.offset_length = lengths[0]*units		# initial offset from base
		self.first_link_length = lengths[1]*units		# first link
		self.second_link_length = lengths[2]*units		# second link
		self.third_link_length = lengths[3]*units		# third link
		self.total_arm_length = sum([self.offset_length,
									 self.first_link_length,
									 self.second_link_length,
									 self.third_link_length])

		self.x_joints = []
		self.y_joints = []
		self.z_joints = []

		self.p_joints = []
		# this is to populate x_joints, y_joints, z_joints and p_joints
		self.forward_kinematics()

	def __repr__(self):
		'''
		Returns a string representation of the class.
		This is what is returned when you type print [DisplayArm]
		'''
		return '{} (x{:.2f} y{:.2f} z{:.2f})'.format(
			self.__class__.__name__,
			self.x_joints[3],
			self.y_joints[3],
			self.z_joints[3])


	def angles(self):
		"""
		This function returns the angles from the last time the inverse kimematics function was
		called. If the function has not been called, the function will return zeros.

		Parameters
		----------
			NONE
		Returns
		-------
		q1 : scalar
			angle of the base
		q2 : scalar
			angle that the first joint makes with the base
		q3 : scalar
			angle that the second joint makes with the first
		q4 : scalar
			angle that the third joint makes with the second

		>>> return [q1,q2,q3,q4]
		"""

		return [self.q1, self.q2, self.q3, self.q4]


	@staticmethod
	def xz_rotation(z_rot, z_tran, x_tran, x_rot):
		"""
		Computes the denavit-Hartenberg rotation matrix.
		This essentailly is a rotation around the x and z axis.


		Perameters
		----------
		z_rot : scalar
			rotation around the z axis
		z_tran : scalar
			translation in the z axis
		x_tran : scalar
			translation in the x axis
		x_rot : scalar
			rotation around the x axis


		Returns
		-------
		out : array_like
			Tranformation maxtrix of how to acheive the desired
			translation and rotation. This array is 4x4.


		More Information
		----------------
		More information can be found here
		https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
		"""
		# pylint: disable=line-too-long
		return np.array([[np.cos(z_rot), -np.cos(x_rot)*np.sin(z_rot), np.sin(x_rot)*np.sin(z_rot), x_tran*np.cos(z_rot)],
					 [np.sin(z_rot), np.cos(x_rot)*np.cos(z_rot), -np.sin(x_rot)*np.sin(z_rot), x_tran*np.sin(z_rot)],
					 [0, np.sin(x_rot), np.cos(x_rot), z_tran],
					 [0, 0, 0, 1.0]])



	def inverse_kinematics(self, goal, tol_limit=0.05, max_iterations=100):
		'''
		Preforms Inverse Kinematics on the arm using the known lengths and last positions.
		This uses the FABRIK method.
		See the more information section for information on the FABRIK method.


		Parameters
		----------
		goal : arraylike or Point
			desired location in the format of [x,y,z]
		tol_limit : scalar (optional)
			tolerance between the desired location and actual one after iteration
		max_iterations : scalar (optional)
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
		x_joints : array_like
			x points for all the joint locations
		y_joints : array_like
			y points for all the joint locations
		z_joints : array_like
			z points for all the joint locations


		More Information
		----------------
		Overall algorithim:
			solve for unit vectors going backwards from the desired point to the original point
			of the joints, then forwards from the origin, see youtube video
			https://www.youtube.com/watch?v=UNoX65PRehA&t=817s

		'''
		if isinstance(goal, list):
			if len(goal) != 3:
				raise IndexError("goal unclear. Need x,y,z coordinates in Point or list form.")
			goal = Point(goal[0], goal[1], goal[2])

		# Find base rotation
		# Initial angle of where end effector is
		initial_base_roation = np.arctan2(self.y_joints[-1], self.x_joints[-1])

		# Desired angle
		# arctan2(y,x)
		self.q1 = np.arctan2(goal.y, goal.x)

		# Base rotation
		base_rotation = self.q1-initial_base_roation

		# Base rotation matrix about z
		z_rot = np.array([[np.cos(base_rotation), -np.sin(base_rotation), 0.0],
						  [np.sin(base_rotation), np.cos(base_rotation), 0.0],
						  [0.0, 0.0, 1.0]])

		# Rotate the location of each joint by the base rotation
		# This will force the FABRIK algorithim to only solve
		# in two dimensions, else each joint will move as if it has
		# a 3 DOF range of motion

		point4 = Point(np.dot(z_rot, [self.x_joints[3], self.y_joints[3], self.z_joints[3]]))
		point3 = Point(np.dot(z_rot, [self.x_joints[2], self.y_joints[2], self.z_joints[2]]))
		point2 = Point(np.dot(z_rot, [self.x_joints[1], self.y_joints[1], self.z_joints[1]]))
		point1 = Point(np.dot(z_rot, [self.x_joints[0], self.y_joints[0], self.z_joints[0]]))

		# store starting point of the first joint
		starting_point1 = point1

		iterations = 0

		# Make sure the desired x,y,z point is reachable
		if Vector.magnitude(goal) > self.total_arm_length:
			print ' desired point is likely out of reach'

		for _ in range(1, max_iterations+1):

			# backwards
			point3 = Vector.project_along_vector(goal, point3, self.third_link_length)
			point2 = Vector.project_along_vector(point3, point2, self.second_link_length)
			point1 = Vector.project_along_vector(point2, point1, self.first_link_length)

			# forwards
			point2 = Vector.project_along_vector(point1, point2, self.first_link_length)
			point3 = Vector.project_along_vector(point2, point3, self.second_link_length)
			point4 = Vector.project_along_vector(point3, goal, self.third_link_length)

			# Solve for tolerance between iterated point and desired x,y,z,
			tol = point4 - goal

			# Make tolerance relative to x,y,z
			tol = Vector.magnitude(tol)

			iterations = iterations+1

			# Check if tolerance is within the specefied limit
			if tol < tol_limit:
				break

		# Re-organize points into a big matrix for plotting elsewhere
		self.p_joints = np.transpose([starting_point1.as_array(), point2.as_array(), point3.as_array(), point4.as_array()])

		self.x_joints = self.p_joints[0]
		self.y_joints = self.p_joints[1]
		self.z_joints = self.p_joints[2]


		# Return the joint angles by finding the angles with the dot produt
		vector21 = Vector(point2 - point1)
		vector32 = Vector(point3 - point2)
		vector43 = Vector(point4 - point3)

		# returns -pi to pi
		self.q2 = np.arctan2(vector21.end.z, Vector.magnitude([vector21.end.x, vector21.end.y]))

		# Negative sign because of dh notation, a rotation away from the previous link
		# and towards the x-y plane is a negative moment about the relative z axis.
		# the relative z axis of each link is out of the page if looking at the arm
		# in 2D
		# the x axis in dh convention is typically along the link direction.

		self.q3 = -1.0*vector21.angle(vector32)
		self.q4 = -1.0*vector32.angle(vector43)

		return [self.q1, self.q2, self.q3, self.q4, self.x_joints, self.y_joints, self.z_joints]

	def forward_kinematics(self, *args, **kwargs):
		'''
		Preforms forward kinematics. This uses known angles to compute the resulting end effector
		location. If this is called with no parameters then
		the stored values for q1-q4 and lengths are used.


		Parameters
		----------
		q1 : scalar (optional)
			angle of the base
		q2 : scalar (optional)
			angle that the first joint makes with the base
		q3 : scalar (optional)
			angle that the second joint makes with the first
		q4 : scalar (optional)
			angle that the third joint makes with the second
		offset_length : scalar (optional)
			offset joint of the first link
		first_link_length : scalar (optional)
			length of the first link
		second_link_length : scalar (optional)
			length of the second link
		third_link_length : scalar (optional)
			length of the third link

		Returns
		-------
		x_joints : array_like
			x points for all the joint locations
		y_joints : array_like
			y points for all the joint locations
		z_joints : array_like
			z points for all the joint locations

		More Information
		----------------
		More information on forward kinematics can be found here
		https://en.wikipedia.org/wiki/Forward_kinematics
		'''
		# pylint: disable=too-many-instance-attributes
		# Use forward kinmatics to move robot
		# Initial forward kinematics

		q1 = self.q1
		q2 = self.q2
		q3 = self.q3
		q4 = self.q4

		offset_length = self.offset_length
		first_link_length = self.first_link_length
		second_link_length = self.second_link_length
		third_link_length = self.third_link_length

		if len(args) >= 1:
			q1 = args[0]
		if len(args) >= 2:
			q2 = args[1]
		if len(args) >= 3:
			q3 = args[2]
		if len(args) >= 4:
			q4 = args[3]
		if len(args) >= 5:
			offset_length = args[4]
		if len(args) >= 6:
			first_link_length = args[5]
		if len(args) >= 7:
			second_link_length = args[6]
		if len(args) >= 8:
			third_link_length = args[7]

		if len(kwargs) > 0:
			if kwargs.has_key['q1']:
				q1 = kwargs['q1']
			if kwargs.has_key['q2']:
				q2 = kwargs['q2']
			if kwargs.has_key['q3']:
				q3 = kwargs['q3']
			if kwargs.has_key['q4']:
				q4 = kwargs['q4']
			if kwargs.has_key['offset_length']:
				offset_length = kwargs['offset_length']
			if kwargs.has_key['first_link_length']:
				first_link_length = kwargs['first_link_length']
			if kwargs.has_key['second_link_length']:
				second_link_length = kwargs['second_link_length']
			if kwargs.has_key['third_link_length']:
				third_link_length = kwargs['third_link_length']

		# Create transformation matrix from 0 to 1
		t10 = DisplayArm.xz_rotation(q1, offset_length, 0, np.pi/2)
		# Create transformation matrix from 2 to 1
		t21 = DisplayArm.xz_rotation(q2, 0, first_link_length, 0)
		# Create transformation matrix from 3 to 2
		t32 = DisplayArm.xz_rotation(q3, 0, second_link_length, 0)
		# keep q4 constant
		t43 = DisplayArm.xz_rotation(q4, 0, third_link_length, 0)

		t20 = np.dot(t10, t21)
		t30 = np.dot(t20, t32)
		t40 = np.dot(t30, t43)	# Transformation matrix from end effector to the global frame

		self.p_joints = np.transpose([t10[0:3, 3], t20[0:3, 3], t30[0:3, 3], t40[0:3, 3]])

		self.x_joints = self.p_joints[0]
		self.y_joints = self.p_joints[1]
		self.z_joints = self.p_joints[2]

		return [self.x_joints, self.y_joints, self.z_joints]


if __name__ == '__main__':
	# animated example of how the arm moving around
	arm = DisplayArm([0.001, 3, 2, 1.5],
					 units=0.3048,
					 q1=45.0/360*2*np.pi,
					 q2=60.0/360*2*np.pi,
					 q3=-30.0/360*2*np.pi,
					 q4=-30.0/360*2*np.pi)

	import matplotlib.pyplot as plt
	#pylint: disable=unused-import
	from mpl_toolkits.mplot3d import axes3d
	import matplotlib.animation as animation

	fig = plt.figure("Arm Demo")
	ax = fig.add_subplot(111, projection='3d')

	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	plt.show()

	# Time matrix for example helix path
	n_cycles = 3
	T = 1.0

	dt = T/100.0
	nt = int(round(n_cycles*T/dt))
	t = np.array(range(0, (nt-1), 1))*dt

	x = 1.7*np.cos(2*np.pi*t+np.pi/8)
	y = 1.7*np.sin(2*np.pi*t+np.pi/8)
	z = 1.05-.5*t

	def animate(index):
		'''
		This fucntion is called by the animation function of matplotlib. It is called between
		each frame and is where the new data is generated.

		Parameters
		----------
			index : integer
				this is the location in the array that we want to pull the information from.

		Returns
		-------
			None

		More Information
		----------------
			More information on the animation module can be found at
			https://matplotlib.org/api/animation_api.html
		'''

		# find angles for x, y, and z
		[_, _, _, _, x_joints, y_joints, z_joints] = arm.inverse_kinematics([x[index], y[index], z[index]])

		# clear and plot the data
		ax.clear()
		plt.hold(True)
		ax.plot3D(x_joints, y_joints, z_joints, color='b', label='links')
		ax.plot3D(x, y, z, color='y', label='path')
		ax.scatter3D(x_joints[0], y_joints[0], z_joints[0], color='g', label='p1')
		ax.scatter3D(x_joints[1], y_joints[1], z_joints[1], color='r', label='p2')
		ax.scatter3D(x_joints[2], y_joints[2], z_joints[2], color='m', label='p3')
		ax.scatter3D(x_joints[3], y_joints[3], z_joints[3], color='k', label='p4')
		#ax.scatter3D(0,0,0							  ,color='k')
		ax.legend()
		ax.plot3D([-.5, .5], [0, 0], [0, 0], color='k')
		ax.plot3D([0, 0], [-.5, .5], [0, 0], color='k')

		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.set_zlabel('z')

		ax.grid(True)
		plt.hold(False)
		plt.show()

	ani = animation.FuncAnimation(fig, animate, range(1, nt-1), interval=100)

