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
# This is a webcam video display demo
# Press c to capture a frame

import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while True;

	ret, frame = cap.read()

 	if ret:
	# show the frame
		cv2.imshow("Webcam Video Test", frame)

	key = cv2.waitKey(1) & 0xFF
 

 
	# if the `q` key was pressed, break from the loop
	# if the `c` key was pressed, saved the current frame
	if key == ord("q"):
		break
	elif key == ord('c'):
		filename = "image" + str(count) + ".jpg"
		cv2.imwrite(filename, image)
		count += 1



cap.release()
cv2.destroyAllWindows()
