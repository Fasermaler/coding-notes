
#!/usr/bin/python
# coding=utf-8

# 29-05-2019

# This script was written by:

# ███████╗ █████╗ ███████╗███████╗██████╗ ███╗   ███╗ █████╗ ██╗     ███████╗██████╗ 
# ██╔════╝██╔══██╗██╔════╝██╔════╝██╔══██╗████╗ ████║██╔══██╗██║     ██╔════╝██╔══██╗
# █████╗  ███████║███████╗█████╗  ██████╔╝██╔████╔██║███████║██║     █████╗  ██████╔╝
# ██╔══╝  ██╔══██║╚════██║██╔══╝  ██╔══██╗██║╚██╔╝██║██╔══██║██║     ██╔══╝  ██╔══██╗
# ██║     ██║  ██║███████║███████╗██║  ██║██║ ╚═╝ ██║██║  ██║███████╗███████╗██║  ██║
# ╚═╝     ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝  ╚═╝╚══════╝╚══════╝╚═╝  ╚═╝

# For maintenance or enquiries, contact me at: https://github.com/fasermaler

# Pi Camera Video script with options and frame buffer flush



from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import timestamp
import time
import numpy as np
from PIL import Image


# Set if you'd like the buffer to be cleared
CLEAR_BUFFER = False

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 32


# ======================== ADDITIONAL OPTIONS: UNCOMMENT IF NECESSARY ==========================
camera.exposure_mode = 'off' # Turn off auto exposure compensation
camera.exposure_compensation = -3 # Set the specific level of exposure compensation
camera.drc_strength = 'off' # Sets the dynamic range compression
camera.still_stats = False # Retrieves or sets whether statistics will be calculated from still frames or the prior preview frame
camera.awb_mode = 'off' # Turn off auto white balance
camera.awb_gains = (Fraction(25, 16), Fraction(25,16)) # Set a specific white balance gain
# ==============================================================================================

rawCapture = PiRGBArray(camera, size=(426, 240))
 
# allow the camera to warmup
time.sleep(0.1)

# Initializes the image counting
count = 0
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text


	# Clears the buffer if the user would like it
	if CLEAR_BUFFER:
		for i in range(5): # Clears the 5 frame buffer 
	        frame = img.array

	else:
		image = frame.array
 
	# show the frame
	cv2.imshow("Pi Camera Video Test", image)
	#img2 = Image.fromarray(frame, 'RGB')
	#img2.show()
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	# if the `c` key was pressed, saved the current frame
	if key == ord("q"):
		break
	elif key == ord('c'):
		filename = "image" + str(count) + ".jpg"
		cv2.imwrite(filename, image)
		count += 1




cv2.destroyAllWindows()
camera.close()