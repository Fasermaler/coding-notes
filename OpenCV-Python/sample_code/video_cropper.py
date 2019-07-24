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
# This script allows the user to select a region of interest and then outputs a cropped video


import cv2
import numpy as np

# =========== DEFINE INPUT AND OUTPUT VIDEO PATHS HERE ============
VIDEO_PATH = 'input_vid.mp4'
OUTPUT = 'output_vid.avi'
# =================================================================



cap = cv2.VideoCapture(VIDEO_PATH)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
refPt = []
cropping = False



def click_and_crop(event, x, y, flags, param):
	# grab references to the global variables
	global refPt, cropping
 
	# if the left mouse button was clicked, record the starting
	# (x, y) coordinates and indicate that cropping is being
	# performed
	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]
		cropping = True
 
	# check to see if the left mouse button was released
	elif event == cv2.EVENT_LBUTTONUP:
		# record the ending (x, y) coordinates and indicate that
		# the cropping operation is finished
		refPt.append((x, y))
		cropping = False
 
		# draw a rectangle around the region of interest
		cv2.rectangle(frame, refPt[0], refPt[1], (0, 255, 0), 2)
		cv2.imshow("image", frame)

ret, frame = cap.read()
clone = frame.copy()
cv2.namedWindow('select ROI')
cv2.setMouseCallback('select ROI', click_and_crop)


while True:
	cv2.imshow('select ROI', frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'r' key is pressed, reset the cropping region
	if key == ord("r"):
		frame = clone.copy()
 
	# if the 'c' key is pressed, break from the loop
	elif key == ord("c"):
		roi = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
		out = cv2.VideoWriter(OUTPUT,cv2.VideoWriter_fourcc(*'XVID'), 300, (roi.shape[1], roi.shape[0]))

		break

if len(refPt) == 2:
	while True:
		ret, frame = cap.read()
		if ret:
			roi = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
			cv2.imshow('ROI', roi)
			out.write(roi)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

cap.release()
out.release()

cv2.destroyAllWindows()



