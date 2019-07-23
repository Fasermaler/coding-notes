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
# This is a circumcenter detection demo
# It requires sympy to run


import cv2
import numpy as np
import time
import random 


from PIL import Image
from math import cos
from sympy import Point, Polygon, pi

# =========== INSERT YOUR GRAYSCALE IMAGE HERE ===============
GRAYSCALE_IMAGE = 'test.jpg'
# ============================================================

frame = cv2.imread(GRAYSCALE_IMAGE, 0)
height, width = frame.shape[:2]
centre = (int(width/2), int(height/2))
dX, dY = 0,0

trials = 1
try:
    # Get the array of indices of detected pixels
    thresholded_array = np.argwhere(frame >= 0.3)
    thresholded_list = thresholded_array.tolist()
    print(thresholded_list)

    
    if len(thresholded_list) > trials*3:
    # sets the number of trials before averaging to get the centre
        
        total_centres_X = 0
        total_centres_Y = 0
        hoop_centre = (0,0)
        arr_len_3rd = int(len(thresholded_list) / 3)
        for i in range(trials):
            r1 = random.randrange(0, int(arr_len_3rd/2))

            #r2 = random.randrange(0, arr_len_3rd)
            # rerolls if the same number was rolled
            #while r2 == r1:
            r2 = random.randrange(arr_len_3rd, 2*arr_len_3rd)
            r3 = random.randrange(int(2.5*arr_len_3rd), len(thresholded_list))
            #while r3 == r1 or r3 == r2:
            #r3 = random.randrange(0, len(thresholded_list))
            print(thresholded_list[r1],thresholded_list[r2],thresholded_list[r3])
            current_centre = Polygon(thresholded_list[r1],thresholded_list[r2],thresholded_list[r3]).circumcenter
            print(current_centre)
            total_centres_X += int(current_centre.y)
            total_centres_Y += int(current_centre.x)
            cv2.circle(frame, (thresholded_list[r1][1], thresholded_list[r1][0]), 5, (0, 0, 255), -1)
            cv2.circle(frame, (thresholded_list[r2][1], thresholded_list[r2][0]), 5, (0, 0, 255), -1)
            cv2.circle(frame, (thresholded_list[r3][1], thresholded_list[r3][0]), 5, (0, 0, 255), -1)
            
        cX = int(total_centres_X / trials)
        cY = int(total_centres_Y / trials)

        print(cX,cY)
except:
    print("no circle detected")
 
# put text and highlight the center
try:
    cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
    cv2.line(frame, centre, (cX, cY), (255,0,0), 2)

#cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    dX = cX - centre[0] 
    dY = centre[1] - cY
    cv2.putText(frame, ("(" + str(dX) + ", " + str(dY) + " )"), (centre[0] - 20, centre[1] - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

except:
    print("No centre detected")





cv2.imwrite('output.jpg', img_rec_red2)