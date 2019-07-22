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

# This is an ROI selection demo
# Instructions can be found on the utility itself

import numpy as np
import cv2



# Defines the font to be used
font = cv2.FONT_HERSHEY_SIMPLEX

# Initializes the reference points array
ref_points = []

# Mouse event function
def mouse_selection(event, x, y, flags, params):
	global ref_points

	if (event == cv2.EVENT_LBUTTONDOWN) and (len(ref_points) < 4):
		ref_points.append((x,y))


cap = cv2.VideoCapture(0)

# Skips the first 49 frames due to initialization
for i in range(50):
	ret, frame = cap.read()

cap.release()

frame_width = frame.shape[0]
# Resize the frame if the resolution is too large 
if frame_width > 1000:
	frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)

# Creates the padding around the image
bordersize = 200
orig_border = cv2.copyMakeBorder(frame, 
								 top=bordersize, 
								 bottom=bordersize, 
								 left=bordersize, 
								 right=bordersize, 
								 borderType=cv2.BORDER_CONSTANT, 
								 value=[96, 96, 96])

row, col = orig_border.shape[:2]

cv2.namedWindow("ROI Selector")
cv2.setMouseCallback("ROI Selector", mouse_selection)

# Status variable for on-screen status printout
status = ""

while True:
	# ensures that every frame gets refreshed 
	frame = orig_border.copy()

	# On-screen Keyboard legend
	cv2.putText(frame, 'ROI Selection Demo written by Fasermaler', (10,30), font, 1, (0, 128 ,255),2, cv2.LINE_AA)
	cv2.putText(frame, "Keyboard Shortcuts:", (10,50), font, 0.5, (0, 255 ,255),2, cv2.LINE_AA)
	cv2.putText(frame, "U - Undo", (10,70), font, 0.5, (0, 255 ,255),2, cv2.LINE_AA)
	cv2.putText(frame, "Esc - Clear", (10,90), font, 0.5, (0, 255 ,255),2, cv2.LINE_AA)
	cv2.putText(frame, "S - Save", (10,110), font, 0.5, (0, 255 ,255),2, cv2.LINE_AA)
	cv2.putText(frame, "Q - Quit", (10,130), font, 0.5, (0, 255 ,255),2, cv2.LINE_AA)

	# Draw lines with each point clicked
	if len(ref_points) > 1:
		cv2.line(frame, ref_points[0], ref_points[1], (0, 0, 255), 2)
	if len(ref_points) > 2:
		cv2.line(frame, ref_points[1], ref_points[2], (0, 0, 255), 2)
	if len(ref_points) > 3:
		cv2.line(frame, ref_points[2], ref_points[3], (0, 0, 255), 2)
		cv2.line(frame, ref_points[3], ref_points[0], (0, 0, 255), 2)

	# Correct the reference points due to padding and/or scaling
	corrected_ref_points = []
	for i in range(len(ref_points)):
		if frame_width < 1000:
			corrected_point = (ref_points[i][0] - bordersize), (ref_points[i][1] - bordersize)
		else:
			corrected_point = (ref_points[i][0] - bordersize)*2, (ref_points[i][1] - bordersize)*2
		corrected_ref_points.append(corrected_point)

	# Reflect the selected points on screen
	cv2.putText(frame, 'Selected Points: ' + str(corrected_ref_points), (10,row-10), font, 0.5, (0, 0 ,255),2, cv2.LINE_AA)	
	cv2.putText(frame, status, (col-55,row-10), font, 0.5, (0, 255 ,0),2, cv2.LINE_AA)
	cv2.imshow('ROI Selector', frame)

	#print(ref_points)


	# Various key events
	key = cv2.waitKey(1) & 0xFF

	# Esc key will reset the ref_points list
	if key == 27:
		ref_points = []
		status = "RESET"

	# U key will remove last point selected
	if key == ord('u'):
		ref_points.pop()
		status = "UNDO"

	# S key will save if the ref_points list has 4 points
	if key == ord('s'):
		if len(ref_points) < 4:
			status = "ERROR"
		else:
			# THIS IS WHERE YOUR REF POINTS ARE
			# STORE THEM ELSEWHERE IF YOU NEED THEM
			print("Your selected ROI points are:\n")
			print(str(corrected_ref_points))
			status = "SAVED"

	# Q will exit
	if key == ord('q'):
		break

cv2.destroyAllWindows()

