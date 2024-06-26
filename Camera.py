import threading
import cv2
import time
import Focus
import db
from collections import deque
import numpy as np

class Cam():

	def __init__(self, q_frame = None, cam_width=640, cam_height=480, cam_fps=30):
		self.running = False
		self.focus_val = 0   
		self.is_recording = False	  
		self.fps = 0
		self.q_frame = q_frame
		self.q_len = 10
		self.cam_width = cam_width
		self.cam_height = cam_height
		self.cam_fps = cam_fps
		self.focus_traker_enabled = False
		self.roi_size = 150
		self.roi_x0 = int(cam_width / 2 - self.roi_size)
		self.roi_x1 = int(cam_width / 2 + self.roi_size)
		self.roi_y0 = int(cam_height / 2 - self.roi_size)
		self.roi_y1 = int(cam_height / 2 + self.roi_size)
		self.cam_text = ""
		self.video_file = ''
		conn = db.get_connection()
		self.camera_state = db.CameraState(conn)
		self.commands = db.Commands(conn)
		self.camera_state.start_recording = False
		self.frame_buffer = deque(maxlen=300)  # FIFO buffer for the most recent 300 frames


	def eval_focus(self, img):
		# get ROI and calculate Laplacian transformation
		roi = img[self.roi_y0:self.roi_y1, self.roi_x0:self.roi_x1]
		gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
		fm = cv2.Laplacian(gray, cv2.CV_64F).var()
		self.focus_val = fm
		return fm, roi

	def start(self, nr=0):
		self.run = True
		self.capture_thread = None
		self.nr = nr
		self.capture_thread = threading.Thread(target=self.worker)
		self.capture_thread.start()
		self.FocusZoomController = Focus.ZoomFocus()
		self.focus_zoom_thread = threading.Thread(target=self.FocusZoomController.worker)
		self.focus_zoom_thread.start()

	def stop(self): 
		self.run = False		
		try:
			self.capture_thread.join()
			self.focus_zoom_thread.join()
		except:
			pass
		
		while self.running:
			time.sleep(0.01)

	def set(self, param=None, value=None):
		self.capture.set(param, value)

	def get(self, param=None):
		self.capture.get(param)

	def worker(self):
		self.running = True
		self.capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
		print('Camera Parameters', self.capture.get(0), self.capture.get(1), self.capture.get(2), self.capture.get(3), self.capture.get(4), self.capture.get(5))
		self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
		self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
		self.capture.set(cv2.CAP_PROP_FPS,30)
		self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
		#self.capture.set(cv2.CAP_PROP_MODE, 3)
		self.frame_width = int(self.capture.get(3))
		self.frame_height = int(self.capture.get(4))
		print("Size", self.frame_width, self.frame_height)
		start = time.time()
		while(self.run):
			ret_video = {}
			end = time.time()
			new_frame, img = self.capture.read()

			if not self.is_recording:  		# Append recent frame to the 300 FIFO buffer while not recording to keep store of the last ~10 seconds
				self.frame_buffer.append(img)

			if self.camera_state.start_recording and not self.is_recording: # If we start recording, create video buffer and create video file
				self.is_recording = True
				print("Start recording with timeStamp {}".format(time.time()))
				self.video_file = cv2.VideoWriter(str(int(time.time()))+'.avi', 0, cv2.VideoWriter_fourcc(*'MJPG'), 25, (640,480))
				self.videobuffer = []
			if self.is_recording and not self.camera_state.start_recording:	# When we stop recording, save the first 300 frames from the FIFO buffer to the video file and then the remaining video frames
				self.is_recording = False

				for f in self.frame_buffer:
					self.video_file.write(f)
				for f in self.videobuffer:
					self.video_file.write(f)
     
				self.video_file.release()
				print("Recording Session Finished")
			
			if self.is_recording and new_frame:
				self.videobuffer.append(img)
    			#self.videobuffer = np.append(self.videobuffer,  img, axis=None)
				#self.video_file.write(img)

			elapsed = end-start
			if elapsed > 0:
				self.fps = 1 / (elapsed)
			else:
				self.fps = 0
			start = end
   		
			if self.camera_state.focus_tracker:
				self.focus_val, _ = self.eval_focus(img)
				self.camera_state.image_focus = self.focus_val
     
			if not self.is_recording or self.is_recording:
				frame = cv2.imencode('.jpg', img)[1].tobytes() 
				self.camera_state.image = (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
		
		self.capture.release()
		self.running = False

def main(d):
	c = Cam()

	print("Starting cam")
	c.start()

	print("Waiting for camera")
	while c.fps == 0:
		time.sleep(0.1) # should be implemented with queue/signals but good enough for testing
	print("Cam is operational")
 
	try:
		while True:
			time.sleep(0.01)
			if not c.running:
					break
	except KeyboardInterrupt:
		pass

	print("Stopping camera")
	c.stop()