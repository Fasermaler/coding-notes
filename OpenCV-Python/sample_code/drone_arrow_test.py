

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

# Full unedited arrow flight test script


from __future__ import print_function
import time
import math
import thread

# Dk imports
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

# Mux and TOF imports
import I2CMultiplexer
import VL53L1X

# CV imports
import cv2
import numpy as np


from picamera.array import PiRGBArray
from picamera import PiCamera
from fractions import Fraction
from PIL import Image

import random 
from sympy import Point, Polygon, pi


direction = None
#cap = cv2.VideoCapture(0)
camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 24
camera.exposure_mode = 'auto'
camera.exposure_compensation = -3
camera.drc_strength = 'off'
camera.still_stats = False

camera.awb_mode = 'off'
camera.awb_gains = (Fraction(167, 103), Fraction(27,16))

rawCapture = PiRGBArray(camera, size=(426, 240))

out = cv2.VideoWriter(str(time.time()) + ".avi",cv2.VideoWriter_fourcc('M','J','P','G'), 10, (426, 240))



# allow the camera to warmup
time.sleep(0.1)
# Connect to Vehicle
connection_string = '/dev/ttyUSB0'
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Global variables for distance:
distance_in_mm_N = 0 # North Sensor
distance_in_mm_S = 0 # South Sensor
distance_in_mm_E = 0 # East Sensor
distance_in_mm_W = 0 # West Sensor
distance_in_mm_45 = 0 # 45 degree south east sensor

dX = 0
dY = 0
quit = False

#Create an I2C Multiplexer object, the address of I2C Multiplexer is 0X70
I2CMulti = I2CMultiplexer.I2CMultiplexer(0x70)  
# Init TOF obj
tof = VL53L1X.VL53L1X()
 # STarts the TOFs on their respective ports
try:
    # for i in [0,2,4,6]:
    for i in [0,1,2,7,3]:
        I2CMulti.selectPort(i)
        tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof.open() # Initialise the i2c bus and configure the sensor
        tof.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
except:
    print("port init failed")

def detect_arrow():
    global direction
    for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        for i in range(5): # Clears the 5 frame buffer 
            frame = img.array
        height, width = frame.shape[:2]
        centre = (int(width/2), int(height/2))

        b_channel = np.array(frame[:,:,0]).astype('float')
        g_channel = np.array(frame[:,:,1]).astype('float')
        r_channel = np.array(frame[:,:,2]).astype('float')

        bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
        img_rec_red2 = np.subtract(r_channel,((b_channel + g_channel)/ 2))
        #img_rec_red2 = np.divide(r_channel, 255)
        img_rec_red2 = np.divide(img_rec_red2,255) 
        #img_rec_red2 = np.square(img_rec_red2)
        img_rec_red2[img_rec_red2 < 0.3] = 0
        img_rec_red2 = img_rec_red2 * 255
        img_rec_red2 = np.floor(img_rec_red2).astype('uint8')



        minimumArea = 30
        try:
            #gray = cv2.cvtColor(img_rec_red,cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(img_rec_red2,50,150,apertureSize = 3)
            cv2.imshow("edges", edges)
            lines = cv2.HoughLines(edges,1,np.pi/40,40)

            
            # convert the grayscale image to binary image
            #ret,thresh = cv2.threshold(img_rec_red,127,255,0)
            #cv2.imshow('thresh', thresh)
             
            # calculate moments of binary image
            im2, contours, hierarchy = cv2.findContours(img_rec_red2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
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
                
                
                

            
            
            for line in lines:
                print(line)
                for rho,theta in line:
                    if theta > 0.698 and theta < 1.047:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho

                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))


                        x_cood = int(-(rY - (rho/b)) / (a/b))
                        print(x_cood,rY)
                    
                        cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
                        cv2.circle(frame, (x_cood, rY), 5, (0,255,255), -1)
                    elif theta > 2.269 and theta < 2.443:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
                        x_cood = int(-(rY - (rho/b)) / (a/b))
                        print(x_cood,rY)
                        cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
                        cv2.circle(frame, (x_cood, rY), 5, (0,255,255), -1)
                    else:
                        x_cood = rX
                        # ensures that no direction is given if arrow point cannot be detected
                    #cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
                    
            
            cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            left_discrepancy = rX - x_cood
            print(left_discrepancy)

            confirmation_threshold = 5 # threshold to be met before confirmation of direction
            l_counter = 0
            r_counter = 0
            direction = None
            if left_discrepancy > 0:
                cv2.putText(frame, "GO LEFT", (rX - 50, rY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                l_counter += 1
                r_counter = 0
                if l_counter == 5:
                    direction = 'left'
            elif left_discrepancy < 0:
                cv2.putText(frame, "GO RIGHT", (rX - 50, rY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                r_counter += 1
                l_counter = 0
                if r_counter == 5:
                    direction = 'right'
            cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        except:
            pass
        #out.write(frame)
        #cv2.imshow('preview', frame)
        #cv2.imshow('rec_red', img_rec_red2)
        if direction != None:
            break
        
        k = cv2.waitKey(1)
        rawCapture.truncate(0)



def detect_circle():
    global dX
    global dY
    for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        for i in range(5): # Clears the 5 frame buffer 
            frame = img.array
        height, width = frame.shape[:2]
        centre = (int(width/2), int(height/2))

        b_channel = np.array(frame[:,:,0]).astype('float')
        g_channel = np.array(frame[:,:,1]).astype('float')
        r_channel = np.array(frame[:,:,2]).astype('float')

        bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
        img_rec_red2 = np.subtract(r_channel,((b_channel + g_channel)/ 2))
        #img_rec_red2 = np.divide(r_channel, 255)
        img_rec_red2 = np.divide(img_rec_red2,255) 
        #img_rec_red2 = np.square(img_rec_red2)
        img_rec_red2[img_rec_red2 < 0.3] = 0
        #dX, dY = 0,0

        trials = 1
        try:
            # Get the array of indices of detected pixels
            thresholded_array = np.argwhere(img_rec_red2 >= 0.3)
            thresholded_list = thresholded_array.tolist()
            #print(thresholded_list)

            
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
                    #print(thresholded_list[r1],thresholded_list[r2],thresholded_list[r3])
                    current_centre = Polygon(thresholded_list[r1],thresholded_list[r2],thresholded_list[r3]).circumcenter
                    #print(current_centre)
                    total_centres_X += int(current_centre.y)
                    total_centres_Y += int(current_centre.x)
                    cv2.circle(frame, (thresholded_list[r1][1], thresholded_list[r1][0]), 5, (0, 0, 255), -1)
                    cv2.circle(frame, (thresholded_list[r2][1], thresholded_list[r2][0]), 5, (0, 0, 255), -1)
                    cv2.circle(frame, (thresholded_list[r3][1], thresholded_list[r3][0]), 5, (0, 0, 255), -1)
                    
                cX = int(total_centres_X / trials)
                cY = int(total_centres_Y / trials)

                #print(cX,cY)
        #except:
            
         
        # put text and highlight the center
        #try:
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.line(frame, centre, (cX, cY), (255,0,0), 2)
       
        #cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
            dX = cX - centre[0] 
            dY = centre[1] - cY
            cv2.putText(frame, ("(" + str(dX) + ", " + str(dY) + " )"), (centre[0] - 20, centre[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #print('Velocities: ' + str(dX) + "," + str(dY))
            border_thresh = 4
            if thresholded_list[0][0]  < border_thresh and (240 - thresholded_list[-1][0]) < 4:
                print("leap condition activated")
                dX = None
                dY = None
                quit = True
        except:

            #print("No centre detected")
            #dX = 0
            #dY = 0
            print("no hoop detected")
            dX = None
            dY = None
            quit = True
            

        out.write(frame)
        k = cv2.waitKey(1)
        rawCapture.truncate(0)
# Arm and rakeoff to specific altitude
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    #Don't try to arm until autopilot is ready
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # while not vehicle.armed == True:
    #     print("Not Armed")
    #     time.sleep(0.4)

    # while not vehicle.armed == True:
    #     vehicle.armed = True
    #     print("Not Armed 2")
    #     time.sleep(0.4)


    #Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude


    # Wait until the vehicle reaches a safe height before processing the goto 
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.rangefinder.distance)
        current_alt = vehicle.rangefinder.distance
        if current_alt > 20:
            current_alt = 0
        print(" Arm state: ", vehicle.armed)
        # Break and return from function just below target altitude.
        if current_alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    print("Initiating GOTO")

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: " + str(remainingDistance))
        if remainingDistance < 0.11: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)

# Sends a velocity to the drone at a rate of 2 Hx
def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.5)



# Sets the Yaw - vehicle will yaw according to the yaw slew rate set in params
# give the vehicle more time (give a 0 velocity vector for x amount of seconds - enough for
# the drone to complete the yaw)
def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

# The following 2 methods allow for the drone attitude to be directly controlled
# the movement is not OF corrected - avoid usage where possible
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000, # Type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        #time.sleep(0.1)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

# Gets the readings from the TOF sensors and updates the distance vars
def get_I2C_readings():
    global distance_in_mm_N
    global distance_in_mm_S
    global distance_in_mm_E
    global distance_in_mm_W
    global distance_in_mm_45
    while(True):

        I2CMulti.selectPort(0)
        distance_in_mm_N = tof.get_distance() # Grab the range in mm

        I2CMulti.selectPort(3)
        distance_in_mm_S = tof.get_distance() # Grab the range in mm

        I2CMulti.selectPort(7)
        distance_in_mm_E = tof.get_distance() # Grab the range in mm

        I2CMulti.selectPort(2)
        distance_in_mm_W = tof.get_distance() # Grab the range in mm
        I2CMulti.selectPort(1)
        distance_in_mm_45 = tof.get_distance() # Grab the range in mm


        #print("Sensor N distance: " + str(distance_in_mm_N) + " \nSensor S distance: " + str(distance_in_mm_S) + "\nSensor E distance: " + str(distance_in_mm_E) + "\nSensor W distance: " + str(distance_in_mm_W))
        time.sleep(0.05)



def calculate_velocity(ground_heading, angle):
    rads = math.radians(angle)
    rads += math.radians(ground_heading)
    if rads > math.radians(360):
        rads -= math.radians(360)
    elif rads < -math.radians(360):
        rads += math.radians(360)
    vel_x = (np.cos(rads) / 5)
    vel_y = (np.sin(rads) / 5)
    return vel_x, vel_y

# Starts TOF readings before takeoff
#thread.start_new_thread(get_I2C_readings, ())

# Starts CV code
thread.start_new_thread(detect_arrow, ())

# Gets vehcle heading on thr ground (this is assumed to be the forward heading)
ground_heading = vehicle.heading


# Takeoff to 1.5m
arm_and_takeoff(1.5)

# Corridor Variables
INCREMENT_DISTANCE = 0.1
CORRIDOR_WIDTH_HALVED = 1300 # in mm
THRESHOLD_DISTANCE = 100
lower_bound = CORRIDOR_WIDTH_HALVED - THRESHOLD_DISTANCE
upper_bound = CORRIDOR_WIDTH_HALVED + THRESHOLD_DISTANCE



#print(str(right_X) + str(right_Y))

VEL_SCALE_Y = 0.016 # velocity scaling factor from openCV
VEL_SCALE_X = 0.001
px_threshold = 10 # sets the threshold before any velocity is taken

print(dX, dY)
# Hoop alignment code
x_aligned = False
y_aligned = False

right_vel_X, right_vel_Y = calculate_velocity(ground_heading, angle)
while direction == None:
    send_global_velocity(0,0,0,5)

if direction == "right":
    send_global_velocity(right_vel_X, right_vel_Y,0,10)
elif direction == "left":
    send_global_velocity(-right_vel_X, -right_vel_Y,0,10)

### SINGLE AXIS ALIGNMENT CODE
# while True:
#     if dX < -px_threshold or dX > px_threshold:
#         # remember, negative means up
#         up_vel = -dX*VEL_SCALE
#         if up_vel > 0.05:
#             up_vel = 0.05
#         elif up_vel < 0.05:
#             up_vel = -0.05
#         send_global_velocity(0,0,(up_vel), 2)
#         send_global_velocity(0,0,0,1) # reset the global vels
#     else:
#         break

# print("x aligned")

# while True:

#     if dY < -px_threshold or dY > px_threshold:
#         right_vel_X = -right_X*dY*VEL_SCALE
#         right_vel_Y = -right_Y*dY*VEL_SCALE
#         if right_vel_X > 0.05:
#             right_vel_X = 0.05
#         elif right_vel_X < -0.05:
#             right_vel_X = -0.05
#         if right_vel_Y > 0.05:
#             right_vel_Y = 0.05
#         elif right_vel_Y < -0.05:
#             right_vel_Y = -0.05
#         send_global_velocity(right_vel_X,right_vel_Y,0,2)
#         send_global_velocity(0,0,0,1) # reset the global vels
#     else :
#         break

### DOUBLE AXIS ALIGNMENT
# up_vel, right_vel_X, right_vel_Y = 0,0,0

# fwd_X, fwd_Y = calculate_velocity(ground_heading, 0)
# right_X, right_Y = calculate_velocity(ground_heading, 90)


# forward_scale = 3.8

# stab_seconds_X = 0
# stab_seconds_Y = 0
# stab_threshold = 1
# loop_count = 0
# #while (not x_aligned) or (not y_aligned):
# while True:  

#     if dX == None or quit == True:
#         print("hoop not detected")
#         send_global_velocity(fwd_X*7,fwd_Y*7,0,3)
#         send_global_velocity(0,0,0,1)
#         break
#     line_d = (dX**2 + dY**2)**0.5

#     if line_d == 0:
#         fwd_x, fwd_y = calculate_velocity(ground_heading, 0)
#         send_global_velocity(fwd_X,fwd_Y,0,2)
#         send_global_velocity(0,0,0,1)

#     total_scale = forward_scale * (1-(line_d / 244.5)) 
#     print(dX, dY)
#     if dX < -px_threshold or dX > px_threshold:
#         x_aligned = False
#         up_vel =  round((-dX*VEL_SCALE_X), 3)
#         if up_vel > 0.1:
#             up_vel = 0.1
#         elif up_vel < -0.1:
#             up_vel = -0.1
#         stab_seconds_X = 0
#     else:
#         if stab_seconds_X == stab_threshold:
#             x_aligned = True
#         else:
#             x_aligned = False
#             stab_seconds_X += 1
#         up_vel = 0

#     if loop_count % 2 == 0:    
#         if dY < -px_threshold or dY > px_threshold:
#             y_aligned = False
#             #angle = math.degrees(np.arctan2(dY , total_scale ))
#             #right_vel_X, right_vel_Y = calculate_velocity(ground_heading, angle)
            
#             right_vel_X = right_X * VEL_SCALE_Y * dY
#             right_vel_Y = right_Y * VEL_SCALE_Y * dY
#             stab_seconds_Y = 0
#         else:
#             if stab_seconds_Y == stab_threshold:
#                 y_aligned = True
#             else:
#                 y_aligned = False
#                 stab_seconds_Y += 1
#             right_vel_X = 0
#             right_vel_Y = 0
#     else:
#         print("sending drone forward")
#         send_global_velocity(fwd_X*total_scale,fwd_Y*total_scale,0,2)
#         send_global_velocity(0,0,0,1)
#         print("velocity: " + str(fwd_X*total_scale) + " : " + str(fwd_Y*total_scale) + " : 0")
#     print("alignment x: " + str(x_aligned))
#     print("alignment y: " + str(y_aligned))
#     print("velocity: " + str(right_vel_X) + " : " + str(right_vel_Y) + " : " + str(up_vel))

#     send_global_velocity(-right_vel_X,-right_vel_Y,up_vel,2)
#     send_global_velocity(0,0,0,1) # reset the global vels

#     loop_count += 1


# print("Fully Aligned")
# send_global_velocity(0,0,0,10) # reset the global vels
# condition_yaw(90, True)
# condition_yaw(-90, True)
        






print("Landing")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()






I2CMulti.i2c.write_byte(0x70,0) # how it closes?
tof.stop_ranging() # Stop ranging

out.release()







