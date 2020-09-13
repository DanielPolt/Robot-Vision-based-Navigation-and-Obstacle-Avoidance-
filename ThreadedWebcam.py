# This class is a wrapper for the camera which adds threading functionality.
# See https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/ for more details.

from threading import Thread
import cv2 as cv

class ThreadedWebcam:
	def __init__(self, src=0, name="ThreadedWebcam"):
		# Initialize camera with a specified resolution.
		# It may take some experimenting to find other valid resolutions,
		# as the camera may end up displaying an incorrect image.
		# Alternatively, frames can be resized afterwards using the resize() function.
		self.stream = cv.VideoCapture(src)
		self.stream.set(cv.CAP_PROP_FRAME_WIDTH, 640)
		self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
		#self.stream.set(cv.CAP_PROP_BUFFERSIZE, 1)
		
		if not self.stream.isOpened():
			print("Failed to open camera!")
			exit()
		else:
			print("Threaded webcam started.")
		
		(self.grabbed, self.frame) = self.stream.read()
		
		self.name = name
		self.stopped = False
	
	# Starts the camera thread.
	def start(self):
		t = Thread(target=self._update, name=self.name, args=())
		t.daemon = True
		t.start()
		return self
	
	# Returns the latest camera frame.
	def read(self):
		return self.frame
	
	# Stops the camera thread.
	def stop(self):
		self.stopped = True
	
	# Private function that constantly reads the camera stream.
	# Do not call this function externally.
	def _update(self):
		while not self.stopped:
			(self.grabbed, self.frame) = self.stream.read()
