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

# This script splits a frame into 4 and returns the cropped images as a cropped array
import cv2


def split_into_four(frame):
		

		Width, Height = frame.shape[1], frame.shape[0]	
		crop_h, crop_w = int(Height/2), int(Width/2)
 		
 		# Get the 4 cropped frames and put them in an array
		crop1 = frame[0:crop_h, 0:crop_w]
		crop2 = frame[0:crop_h, crop_w:Width]
		crop3 = frame[crop_h:Height, 0:crop_w]
		crop4 = frame[crop_h:Height, crop_w:Width]
		crop_array = [crop1, crop2, crop3, crop4]


		return crop_array
