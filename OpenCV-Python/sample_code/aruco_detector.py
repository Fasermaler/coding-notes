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

# This is a class implementation of aruco detection


import cv2
import numpy as np

import cv2.aruco as aruco


class aruco_detector:

	# aruco format determines the type of aruco markers being used (number of bits)
	# allowable values are 4, 5 or 6
	def __init__(self, aruco_format=6):

		# Checks aruco_format, will always default to 6x6 aruco if funny values are set
		if aruco_format == 4:

			self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
		
		elif aruco_format == 5:

			self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
		
		else:

			self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

		# create aruco params
		self.parameters = aruco.DetectorParameters_create()

		#font for displaying text (below)
		self.font = cv2.FONT_HERSHEY_SIMPLEX 

	# takes a frame (default is full color frame)
	# returns the list of corners and ids
	def return_aruco_ids(self, frame, full_color=True):

		if full_color:

			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


		else:

			gray = frame.copy()
			# gets the corners and ids of aruco markers
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)


		return corners, ids

	# subroutine to draw the markers on a selected frame
	# takes in corners, ids and text to be drawn
	# set id_flag or text_flag to True if they need to be drawn
	def draw_markers(self, frame, corners, ids, text=None, id_flag=False, text_flag=False):
		if ids == None:

			print("[ERROR] No Markers to draw")
		aruco.drawDetectedMarkers(frame, corners)
		print(corners)


		if id_flag:
			for i in range(len(ids)):
				cv2.putText(frame, ids[i], (corners[i]), self.font, 1, (0,255,0),2,cv2.LINE_AA)

		if text_flag:
			for i in range(len(text)):
				cv2.putText(frame, text[i], ((int(corners[i][0][0][0]), int(corners[i][0][0][1]))), self.font, 1, (0,255,0),1,cv2.LINE_AA)



## Test code ##

if __name__ == '__main__':
	image = cv2.imread("test.jpg")
	a_detector = aruco_detector()
	corner, ids = a_detector.return_aruco_ids()
	a_detector.draw_markers(image, corner, ids)

	cv2.imshow('image', image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
	    break

	cv2.destroyAllWindows()