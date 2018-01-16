# -*- coding: utf-8 -*-
"""
Created on Tue Aug 01 11:22:00 2017

@author: joell

"""
import settings
import sys
import ctypes
import time
import json
from operator import itemgetter, attrgetter
from itertools import count, starmap
from pyglet import event
# W0312 disables the warnings for tabs vs spaces
# C0330 disables the 'wrong hanging indent' warning

#pylint: disable=W0312,C0330

class XINPUT_GAMEPAD(ctypes.Structure):
	"""
	structs according to http://msdn.microsoft.com/en-gb/library/windows/desktop/ee417001%28v=vs.85%29.aspx
	This is a class that holdes a ctype structure of how big the buttons and triggers are.
	"""
	_fields_ = [
		('buttons', ctypes.c_ushort),# wButtons
		('left_trigger', ctypes.c_ubyte),# bLeftTrigger
		('right_trigger', ctypes.c_ubyte),# bLeftTrigger
		('l_thumb_x', ctypes.c_short),# sThumbLX
		('l_thumb_y', ctypes.c_short),# sThumbLY
		('r_thumb_x', ctypes.c_short),# sThumbRx
		('r_thumb_y', ctypes.c_short),# sThumbRy
	]

class XINPUT_STATE(ctypes.Structure):
	"""
	This is a class that holds a ctype struct of how big a packet and a gamepad are
	"""
	_fields_ = [
		('packet_number', ctypes.c_ulong),# dwPacketNumber
		('gamepad', XINPUT_GAMEPAD),# Gamepad
	]


class XINPUT_VIBRATION(ctypes.Structure):
	"""
	This is a class that holdes a ctype struct of how big the left and right motor speeds are.
	"""
	_fields_ = [("wLeftMotorSpeed", ctypes.c_ushort),
				("wRightMotorSpeed", ctypes.c_ushort)]

class bcolors:
	"""
	bcolors
	A small class that provides strings that can change the terminal text to different colors.

	This includes:
		HEADER
		OKBLUE
		OKGREEN
		WARNING
		FAIL
		ENDC
		BOLD
		UNDERLINE

	I think this class is depreciated if the user uses termcolor

	"""
	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'

class XInputJoystick(event.EventDispatcher):

	"""
	XInputJoystick

	A stateful wrapper, using pyglet event model, that binds to one
	XInput device and dispatches events when states change.

	Example:
	controller_one = XInputJoystick(0)

	THIS CLASS IS TAKEN FROM https://github.com/r4dian/Xbox-360-Controller-for-Python
	THIS IS ME GIVING CREDIT WHRERE ITS DUE
	"""
	max_devices = 4

	def __init__(self, device_number, normalize_axes=True):
		"""
		XInputJoystick

		A stateful wrapper, using pyglet event model, that binds to one
		XInput device and dispatches events when states change.

		Parameters
		----------
		device_number : Integer
			This is the controller you would like to bind to
		normalize_axes : Boolean (optional)
			defaults to true. This maps the analog outputs between -1 and 1.

		Example:
			>>> controller_one = XInputJoystick(0)
		"""

		values = vars()
		self.button_callbacks = {}
		self.axis_callbacks = {}
		del values['self']
		self.__dict__.update(values)

		super(XInputJoystick, self).__init__()

		self._last_state = self.get_state()
		self.received_packets = 0
		self.missed_packets = 0
		self.device_number = device_number

		# Set the method that will be called to normalize
		#  the values for analog axis.
		choices = [self.translate_identity, self.translate_using_data_size]
		self.translate = choices[normalize_axes]

	def translate_using_data_size(self, value, data_size):
		"""
		normalizes analog data to [0,1] for unsigned data and [-0.5,0.5] for signed data
		"""
		data_bits = 8 * data_size
		return float(value) / (2 ** data_bits - 1)

	def button_attach_callback(self, button, func):
		'''
		Used for attaching a function callback to happen when a button is pressed.

		Parameters
		----------
		Button : Integer
			Which button do you want to attach the callback to.
			Options are:
				1. Up on D-pad
				2. Down on D-pad
				3. Left on D-pad
				4. Right on D-pad
				5. Start button
				6. Back
				7. Left Stick Click
				8. Right Stick Click
				9. Left bumper
				10. Right bumper
				11. Unknown
				12. Unknown
				13. A Button
				14. B Button
				15. X Button
				16. Y button

		func : function
			A function to be called when the button is changed

		Returns
		-------
		None
		'''
		self.button_callbacks[button] = func

	def axis_attach_callback(self, axis, func):
		'''
		This is a small function that is called every time one of the joysticks or
		triggers is moved on an xbox controller

		Parameters
		----------
		axis : string
			This is a string that describes what axis the value is for.
			Current Values are :
			1. left_trigger
			2. right_trigger
			3. l_thumb_x
			4. l_thumb_y
			5. r_thumb_x
			6. r_thumb_y
		func : function
			A function to be called when the axis has changed

		Returns
		-------
		None
		'''
		self.axis_callbacks[axis] = func


	def translate_identity(self, value, data_size=None):
		""" I dont know what this class does. Seems dumb."""
		return value

	def get_state(self):
		"""
		Get the state of the controller represented by this object
		"""
		state = XINPUT_STATE()
		res = xinput.XInputGetState(self.device_number, ctypes.byref(state))
		if res == ERROR_SUCCESS:
			return state
		if res != ERROR_DEVICE_NOT_CONNECTED:
			raise RuntimeError("Unknown error %d attempting to get state of device %d" % (res, self.device_number))
		# else return None (device is not connected)

	def is_connected(self):
		"""
		Parameters
		----------
		None

		Returns
		-------
		out : Boolean
			if the device is connected or not
		"""
		return self._last_state is not None

	@staticmethod
	def enumerate_devices():
		"""
		Returns the devices that are connected

		Parameters
		----------
		None

		Returns
		-------
		out : arraylike
			an array that is size [connected devices] that contain boolean values.
		"""
		devices = list(map(XInputJoystick, list(range(XInputJoystick.max_devices))))
		return [d for d in devices if d.is_connected()]

	def set_vibration(self, left_motor, right_motor):
		"""
		Control the speed of both motors seperately

		Parameters
		----------
		left_motor, right_motor : positive integers
			sets the speed of the motors inside the device. I think the valid range is 0-255?

		Returns
		-------
		None
		"""
		# Set up function argument types and return type
		XInputSetState = xinput.XInputSetState
		XInputSetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_VIBRATION)]
		XInputSetState.restype = ctypes.c_uint

		vibration = XINPUT_VIBRATION(int(left_motor * 65535), int(right_motor * 65535))
		XInputSetState(self.device_number, ctypes.byref(vibration))

	def dispatch_events(self):
		"""
		The main event loop for a joystick. This method checks the state, and if the device is
		connected then it makes sure that the approprate handles get notified.

		Parameters
		----------
		None

		Returns
		-------
		None

		"""
		state = self.get_state()
		if not state:
			raise RuntimeError("Joystick %d is not connected" % self.device_number)
		if state.packet_number != self._last_state.packet_number:
			# state has changed, handle the change
			self.update_packet_count(state)
			self.handle_changed_state(state)
		self._last_state = state

	def update_packet_count(self, state):
		"""
		Keep track of received and missed packets for performance tuning

		Parameters
		----------
		state : Boolean
			if the device is connected or not

		Returns
		-------
		None
		"""
		self.received_packets += 1
		missed_packets = state.packet_number - \
			self._last_state.packet_number - 1
		if missed_packets:
			self.dispatch_event('on_missed_packet', missed_packets)
		self.missed_packets += missed_packets

	def handle_changed_state(self, state):
		"""
		Dispatch various events as a result of the state changing.
		This method is the one that handles making sure the buttons and axis events get called.

		Parameters
		----------
		state : Boolean
			if the device is connected or not

		Returns
		-------
		None

		"""
		self.dispatch_event('on_state_changed', state)
		self.dispatch_axis_events(state)
		self.dispatch_button_events(state)

	def dispatch_axis_events(self, state):
		"""
		Event method to see if any of the axis have changed and if so notify the approprate method.

		Parameters
		----------
		state : Boolean
			if the device is connected or not.
		"""
		# axis fields are everything but the buttons
		axis_fields = dict(XINPUT_GAMEPAD._fields_)
		axis_fields.pop('buttons')
		for axis, type in list(axis_fields.items()):
			old_val = getattr(self._last_state.gamepad, axis)
			new_val = getattr(state.gamepad, axis)
			data_size = ctypes.sizeof(type)
			old_val = self.translate(old_val, data_size)
			new_val = self.translate(new_val, data_size)

			# an attempt to add deadzones and dampen noise
			# done by feel rather than following https://msdn.microsoft.com/en-gb/library/windows/desktop/ee417001%28v=vs.85%29.aspx#dead_zone
			# ags, 2014-07-01
			if ((old_val != new_val and
					(new_val > 0.08 or new_val < -0.08) and
					abs(old_val - new_val) > 0.000000005) or
					(axis == 'right_trigger' or axis == 'left_trigger') and
					new_val == 0 and
					abs(old_val - new_val) > 0.000000005):
				self.dispatch_event('on_axis', axis, new_val)

	def dispatch_button_events(self, state):
		changed = state.gamepad.buttons ^ self._last_state.gamepad.buttons
		changed = self.get_bit_values(changed, 16)
		buttons_state = self.get_bit_values(state.gamepad.buttons, 16)
		changed.reverse()
		buttons_state.reverse()
		button_numbers = count(1)
		changed_buttons = list(filter(itemgetter(0), list(zip(changed, button_numbers, buttons_state))))
		tuple(starmap(self.dispatch_button_event, changed_buttons))

	def dispatch_button_event(self, changed, number, pressed):
		self.dispatch_event('on_button', number, pressed)

	def struct_dict(self, struct):
		get_pair = lambda field_type: (field_type[0], getattr(struct, field_type[0]))
		return dict(list(map(get_pair, struct._fields_)))

	def get_bit_values(self, number, size=32):
		res = list(self.gen_bit_values(number))
		res.reverse()
		# 0-pad the most significant bit
		res = [0] * (size - len(res)) + res
		return res

	def gen_bit_values(self, number):
		number = int(number)
		while number:
			yield number & 0x1
			number >>= 1

	# stub methods for event handlers
	def on_state_changed(self, state):
		pass

	def on_axis(self, axis, value):
		settings.controller.values[axis] = value
		if self.axis_callbacks.has_key(axis):
			self.axis_callbacks[axis](axis, value)

	def on_button(self, button, pressed):
		if self.button_callbacks.has_key(button):
			if pressed == 1:
				self.button_callbacks[button](button, pressed)

	def on_missed_packet(self, number):
		pass


list(map(XInputJoystick.register_event_type, [
	'on_state_changed',
	'on_axis',
	'on_button',
	'on_missed_packet',
]))

ERROR_DEVICE_NOT_CONNECTED = 1167
ERROR_SUCCESS = 0

xinput = ctypes.windll.xinput9_1_0


def joystickWorker():

	"""
	Grab 1st available gamepad, logging changes to the screen.
	L & R analogue triggers set the vibration motor speed.
	"""
	joysticks = XInputJoystick.enumerate_devices()
	device_numbers = list(map(attrgetter('device_number'), joysticks))

	print 'found %d devices: %s' % (len(joysticks), device_numbers)

	if not joysticks:
		sys.stderr.write("No Controllers Found, Exiting\n")
		settings.globalDict['exit'] = True
		return

	j = joysticks[0]
	print 'using %d' % j.device_number
	settings.globalDict['XinputJoystick'] = j

	#@j.event
	#def on_button(button, pressed):
		#print('button', button, pressed)
	#	pass

	#@j.event
	#def on_axis(axis, value):
		#settings.controller.values[axis] = value
		#print axis+":"+str(value)
		#print('axis', axis, value)

	while True:
		j.dispatch_events()
		time.sleep(.01)
		if settings.globalDict['exit']:
			print 'ending the joystick thread'
			return

class XboxControls():
	def __init__(self):
		self.validOptions = ['Forward Speed', 'Reverse Speed', 'X Vector Direction', "None"]
		self.inputs = ['left_trigger', 'right_trigger', 'l_thumb_x', 'l_thumb_y', 'r_thumb_x', 'r_thumb_y']
		# TODO: make values auto assign from self.inputs in a loop
		self.values = {'left_trigger':0, 'right_trigger':0, 'l_thumb_x':0, 'l_thumb_y':0, 'r_thumb_x':0, 'r_thumb_y':0}
		self.profiles = ['linear', 'sigmoid']
		self.controls = {
			'left_trigger':self.validOptions[0],
			'right_trigger':self.validOptions[1],
			'l_thumb_x':self.validOptions[2],
			'l_thumb_y':self.validOptions[-1],
			'steer':self.profiles[0],
			'speed':self.profiles[0]
		}
		self.M = {'M1':0, 'M2':0, 'M3':0, 'M4':0}


	def load(self, fileName):
		with open(fileName, 'r') as fi:
			self.controls = json.loads(fi.readline())
		#print self.controls

	def save(self, fileName):
		with open(fileName, 'w') as fo:
			fo.write(json.dumps(self.controls))

	def setControl(self, controlToSet, whatToSetItTo):
		if whatToSetItTo in self.validOptions:
			if controlToSet in self.inputs:
				self.controls[controlToSet] = whatToSetItTo
		else:
			sys.stderr.write("Unable to set " +str(controlToSet)+ " to " +str(whatToSetItTo)+"\n")
			sys.stderr.flush()

	def linear(self, value, linput=0, uinput=1, loutput=0, uoutput=255):
		temp = ((value-linput)*(uoutput-loutput)/(uinput-linput)+loutput)
		if temp > uoutput:
			temp = uoutput
		elif temp < loutput:
			temp = loutput
		return temp
	def getMotors(self):
		'''
		Assuming motors in this layout
		 FRONT
		M2   M4

		M1   M3

		#front left   2
		#front right  4
		#back  left   1
		#back  right  3

		'''
		self.M = {'M1':0, 'M2':0, 'M3':0, 'M4':0}
		forward = 0
		reverse = 0
		x = 0
		xdir = False

		for key in self.controls:
			if self.controls[key] == self.validOptions[0]:
				forward = self.values[key]
			elif self.controls[key] == self.validOptions[1]:
				reverse = self.values[key]
			elif self.controls[key] == self.validOptions[2]:
				x = self.linear(abs(self.values[key]), -0.5, 0.5, -1, 1)
				xdir = self.values[key] > 0

		for key in self.M:
			self.M[key] = self.linear(max([forward, reverse]))

		if xdir:
			self.M['M4'] = self.M['M4'] - x*self.M['M4']
			self.M['M3'] = self.M['M3'] - x*self.M['M3']
		else:
			self.M['M2'] = self.M['M2'] - x*self.M['M2']
			self.M['M1'] = self.M['M1'] - x*self.M['M1']
		if reverse > forward:
			for key in self.M:
				self.M[key] = -self.M[key]
		return self.M
