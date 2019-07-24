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
# This script contains 2 functions for drawing bounding boxes on frames based on the output of a model
# Convention 0 refers to [x, y, w, h] convention
# Convention 1 refers to [x1, y1, x2, y2] convention

import cv2


def draw_boxes_conv0(frame, boundingboxlist):
	
	for box in boundingboxlist:
		x = box[0]
		y = box[1]
		w = box[2]
		h = box[3]
		cv2.rectangle(frame, (x, y), (int(x + w), int(y + h)), (0,0,255), 3)
	
	return frame

def draw_boxes_conv1(frame, boundingboxlist):
	
	for box in boundingboxlist:
		x1 = box[0]
		y1 = box[1]
		x2 = box[2]
		y2 = box[3]
		cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), 3)
	
	return frame