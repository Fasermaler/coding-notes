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

# Camera undistortion demo
# This is an undistortion demo, the mtx and dist should be obtained from the camera calib process

import cv2

import time
import numpy as np
from PIL import Image


cap = cv2.VideoCapture(0)

mtx = np.array([[  1.31960797e+03,   0.00000000e+00,   1.00916350e+03],
       [  0.00000000e+00,   1.35786213e+03,   6.36593029e+02],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
dist = np.array([[ -3.86818203e-01,   2.81274627e+00,  -2.46548663e-02,
         -1.66810086e-05,  -1.20151138e+01]])
 
while True:

	ret, frame = cap.read()
	
	h,  w = frame.shape[:2]
	newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
	undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
 
	# show the frame
	cv2.imshow("preview", undistort)
	#img2 = Image.fromarray(frame, 'RGB')
	#img2.show()
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
	# elif key == ord('c'):
	# 	filename = "image" + str(count) + ".jpg"
	# 	cv2.imwrite(filename, image)
	# 	count += 1
cv2.destroyAllWindows()