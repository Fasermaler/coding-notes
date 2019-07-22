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

# Red thresholding demo
# Filters red and only red to produce a very strong signal
# Will require calibration and is very suseceptible to lighting changes
# This is not a robust algorithm and should not be treated as such
# However, in specific use cases, it will produce a much clearer signal than hue or other color based methods


# Sets the percentage red threshold, below which a pixel will be eliminated from the final result
PERC_RED_THRESH = 0.3

cap = cv2.VideoCapture(0)


while True:

	ret, frame = cap.read()

	if ret:
		cv2.imshow('original', frame)

		# Gets the individual color channels
		# The format assumed here is bgr
		b_channel = np.array(frame[:,:,0]).astype('float')
		g_channel = np.array(frame[:,:,1]).astype('float')
		r_channel = np.array(frame[:,:,2]).astype('float')

		# Create a base channel where all values are added together
		bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)

		# Take the average of the blue and green channels and subtract it from the red channel
		# This will eliminate white color from the final image
		reduce_white = np.subtract(r_channel,((b_channel + g_channel)/ 2))

		# Find the percentage red by dividing the previous channel by 255
		perc_red = np.divide(reduce_white,255)

		# Eliminate all pixels that fail to meet the minimum percentage red using numpy conditional array value setting
		perc_red[perc_red < PERC_RED_THRESH] = 0

		# Set the values back to within a range of 255 and set the type back to uint8 so that the frame becomes valid for OpenCV processing
		final_image = perc_red * 255
		final_image = np.floor(final_image).astype('uint8')

		cv2.imshow('red filtered', final_image)

		# Run another round of openCV thresholding as required
		ret, th = cv2.threshold(final_image,10,255,cv2.THRESH_BINARY)

		cv2.imshow('thresholded red', th)

	if cv2.waitKey(1) & 0xFF == ord('q'):
    	break
        
cap.release()
cv2.destroyAllWindows()