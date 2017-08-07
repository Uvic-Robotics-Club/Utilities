import socket
import time
import cv2
import numpy as np
import threading

# This class sends images over a provided python socket
class sender:
	s = None
	error = None
	
	#Initializes using a provided socket
	def __init__(self, sokt):
		self.s = sokt
		self.s.settimeout(0.1)
	
	# Sends the provided image. Returns a message if a socket error occurs
	def send(self, img):
		# Encode the image as a String
		# Note: the resulting String doesn't play nice with some of Python's functions
		imgStr = cv2.imencode('.png', img)[1].tostring()
		length = len(imgStr)
		# Break the string into bit-sized pieces and send each piece
		try:
			for i in range(0,length, 512):
				if i + 512 < length:
					toSend = imgStr[i:i+512]
				else:
					toSend = imgStr[i:]
				self.s.send(toSend)
			# Tell the listener that all pieces have been sent
			self.s.send("Image Complete")
			return True
		except socket.timeout:
			self.error = "Error: socket timed out"
			return False
		except socket.error:
			self.error = "Error: socket error (likely has disconnected)"
			return False
	
	# All errors are handled internally so that sender methods don't have to be in
	# try/catch loops. If a send operation fails, it will return false and the
	# error message is available through this method
	def get_error_message(self):
		return self.error


# This class receives images over a provided python socket, provided by a sender object
class listener:
	c = None
	frame = None
	newFrame = False
	running = False
	
	# Creates the object using a provided socket
	# Starts the listener as a thread
	def __init__(self, s):
		self.c = s
		self.c.settimeout(2)
		self.frame = None
		self.start()
	
	# A method that runs continuously to receive and process new images
	def listen_for_input(self):
		while self.running:
			try:
				temp = self.c.recv(1024)
				data = ""
				while not temp == "Image Complete":
					data = data + temp
					temp = self.c.recv(1024)
				nparr = np.fromstring(data, np.uint8)
				self.frame = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
				self.newFrame = True
				print "image received"
			except socket.timeout:
				pass
			except socket.error:
				self.running = False
	
	# Returns the most recent image
	def update(self):
		self.newFrame = False
		return self.frame
	
	# Returns true if a new image has been received
	def has_update(self):
		return self.newFrame
	
	# Tells the listener to stop. This terminates its thread within about 2 seconds
	def stop(self):
		self.running = False
	
	# Starts the listener as a new thread, or returns false if it is already running
	def start(self):
		if self.running:
			return False
		self.running = True
		threading.Thread(target=self.listen_for_input).start()
		return True
		
	# Returns whether or not the listener is currently running
	def is_running(self):
		return self.running

# Some sample code for creating and using the receiver
def main2():
	#Initialize and connect the socket
	s = socket.socket()
	host = socket.gethostname()
	port = 9999
	s.bind((host, port))
	s.listen(5)
	print('[Waiting for connection...]')
	c, addr = s.accept()
	print 'Got connection from', addr
	# Create a new listener with the socket
	testListener = listener(c)
	while True:
		# Display the received image. Update the image to the most recent one
		# when the user presses the '0' key
		if testListener.has_update():
			img = testListener.update()
			cv2.imshow('image', img)
			cv2.waitKey(0)
		else:
			time.sleep(0.1)

# Some sample code for creating and using the sender
# Note that it requires images named "imageX" for X = 1 through 31 (inclusive)
# stored in a folder named "Sample_images" in this directory to run
def main1():
	# Create and connect the socket
	s = socket.socket()
	host = socket.gethostname()
	port = 9999
	while True:
		try:
			s.connect((host, port))
			break
		except:
			pass
	print 'Connected to', host
	# Create the sender object with the socket
	testSend = sender(s)
	while True:
		for i in range(1,4,1):
			img = cv2.imread("Sample_images/image" + str(i) + ".png")
			assert img is not None
			testSend.send(img)
			time.sleep(1)
			

if __name__ == "__main__":
	threading.Thread(target=main1).start()
	main2()