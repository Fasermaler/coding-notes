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

# Contour and centroid Detection Demo


import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while True;

	ret, frame = cap.read()

	if ret:

		img2, contours, hierarchy = cv2.findContours(frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		    for c in contours:
		        if cv2.contourArea(c) > minimumArea:
		            # calculate moments for each contour
		            M = cv2.moments(c)
		            cX = int(M["m10"] / M["m00"])
		            cY = int(M["m01"] / M["m00"])
		            cv2.circle(frame, (cX, cY), 5, (255, 100, 255), -1)
		            cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		            cv2.drawContours(frame, [c], -1, (255, 100, 255), 2)
		            x,y,w,h = cv2.boundingRect(c)
		            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
		            rX = int(x+(w/2))
		            rY = int(y+(h/2))
		            cv2.circle(frame, (rX, rY), 5, (0,255,255), -1)


		cv2.imshow('centroid_test', frame)

		key = cv2.waitKey(1) & 0xFF

		if key == ord("q"):
			break


cap.release()
cv2.destroyAllWindows()
