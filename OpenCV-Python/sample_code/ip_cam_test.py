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

# IP Camera test viewer
# Change the connection URL to the correct URL and go ahead to test your IP camera!


import numpy as np
import cv2
import time


# ================= PUT CONNECTION URL HERE ====================
CONNECTION_URL = 'rtsp://admin:12345@192.168.0.11:554/MediaInput/h264'
# ==============================================================

cap = cv2.VideoCapture(CONNECTION_URL)



count = 0
while(True):
	
	ret, frame = cap.read()
	
	try:
		cv2.imshow('frame', frame)

	except Exception as e:
		print(e)

		# Check if connection failed for 10s before closing
		if count > 10:
			break
		else:
			count+= 1
			time.sleep(1)
			print(count)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()