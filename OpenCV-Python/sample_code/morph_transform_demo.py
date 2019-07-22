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

# This is a morphological transform demo
# It will display the effects of a simple cross kernel with erode, dilate, blackhat and tophat


cap = cv2.VideoCapture(0)

# Feel free to design a new kernel of your choosing
KERNEL = cv2.getStructuringElement(cv2.MORPH_CROSS,(9,9)) # 9x9 cross kernel

# Invert the frame
INVERT = False

# Enable or disable thresholding
# Disabling will mena the image will be converted to gray
THRESH = True


while True:

	ret, frame = cap.read()

	if ret:
		cv2.imshow('original', frame)

		if THRESH:
			ret, gray_img = cv2.threshold(frame,10,255,cv2.THRESH_BINARY)
		else:
			gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		if INVERT:
			gray_img = cv2.bitwise_not(gray_img, gray_img)


		cv2.imshow('thresholded', gray_img)

		eroded_frame = cv2.erode(gray_img, kernel) # Erode transform
		dilated_frame = cv2.dilate(gray_img, kernel) # Dilate transform

		tophat = cv2.morphologyEx(gray_img, cv2.MORPH_TOPHAT, kernel) # Top Hat transform
		blackhat = cv2.morphologyEx(gray_img, cv2.MORPH_BLACKHAT, kernel) # Black Hat transform


		cv2.imshow('eroded', eroded_frame)
		cv2.imshow('dilated', dilated_frame)
		cv2.imshow('tophat', tophat)
		cv2.imshow('blackhat', blackhat)



	if cv2.waitKey(1) & 0xFF == ord('q'):
    	break
        
cap.release()
cv2.destroyAllWindows()



