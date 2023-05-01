#!/usr/bin/python3
##############################################################################
# BME 210 2023
# Game Play Skeleton
##############################################################################
##############################################################################
# Urs Utzinger 4/14/2023
##############################################################################

DEBUG = True
DISPLAY = True
TEXT_DEBUG = False

############
# Imports
############

import cv2
import numpy as np
import math
import time
from copy import copy
from picamera2 import Picamera2
import RPi.GPIO as GPIO # import Raspberry Pi input out put
import meArm
from grip23 import WhiteBall
arm = meArm.meArm()
arm.begin(0,0x6f)

start_pin  = 21;
# Switch
switch_pin = 20

##############################################################################
#  Setup
##############################################################################
                     

##############################################################################
# Initialize
##############################################################################

# Initialize Button and Switch
GPIO.setmode(GPIO.BCM)
# Button is input
GPIO.setup(start_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

##############################################################################
# Main Loop
##############################################################################

# Main Loop
#########################################################
stop = False
ready_toThrow = False


while (not stop):
    
    # Read Switch
    start_state  = GPIO.input(start_pin)
    switch_state = GPIO.input(switch_pin)
    str_start    = "pushed" if start_state else "not pushed"
    str_switch   = "Defense" if switch_state else "Attack"
    if TEXT_DEBUG: print("Start is {} and Switch is {}".format(str_start, str_switch))

    if switch_state==1:
        ####################################################
        # Defense
        ####################################################
        cv2.startWindowThread()

        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
        picam2.start()

        process = WhiteBall()

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.25
        color = (0, 255, 0)
        thickness = 1

        width =  320
        height = 240
        startX =   0
        startY = 100
        endX =   320
        endY =   200

        #############
        # meArm
        #############

        from pynput.keyboard import Listener
        import meArm
        import time

        arm = meArm.meArm() # takes inserted data from meArm.py aka calibration data
        arm.begin(0,0x6f) #

        # Idle Position
        xi = 145
        yi =  60
        zi = 110

        # Defense
        #########
        x_offset = 0     # depends on your position in the field
        x_gain   = -1    # depends on the distance and camera angle
        z_offset = 0     # depends on the distance and camera angle
        z_gain   = 0.5   # might need to be same as x_gain but opposite sign
        motor_y  = 180   # defines the position from the goal during defense

        # Throwing
        ##############
        # Start Position
        xs = 145 # x coordinate
        ys =  60 # y coordinate
        zs = 110 # z coordinate
        # End Position
        xe =   0 # x coordinate
        ye = 195 # y coordinate
        ze =  90 # z coordinate

        import collections
        import statistics
        # this will slow response down, perhaps better to not filter
        pos_x = collections.deque(maxlen=3) # if you want to take median of 3 locations. 
        pos_y = collections.deque(maxlen=3)
        pos_x.append(x_offset) # initialize
        pos_y.append(z_offset)

        # Initilize the arm and camera
        arm.gotoPoint(xi,yi,zi) 

        # Main Loop
        stop = False

        while (not stop):

            #####
            # Detect Ball
            #####
            img = picam2.capture_array()
            display_img = cv2.resize(img, ((int)(width), (int)(height)), 0, 0, cv2.INTER_CUBIC)
            # Extract ROI
            proc_img = display_img[startY:endY,startX:endX,:] 
            # Process the image to find ball
            process.process(proc_img)
            # Display ball
            if len(process.ball) > 0:
                x = process.ball[1] + startX
                y = process.ball[2] + startY
                # pos_x = x # no filtering
                # pos_y = y
                pos_x.append(x)
                pos_y.append(y)
                #area = process.ball[3]
                #circ = process.ball[4]  
                #approx = process.ball[5] 
                hsv = cv2.cvtColor(display_img, cv2.COLOR_BGR2HSV)
                cv2.drawMarker(display_img, (x, y),  (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
                txt = "{:3n},{:3n},{:3n}".format(hsv[y,x,0], hsv[y,x,1], hsv[y,x,2])
                cv2.putText(display_img, txt, (x,y), font, fontScale, color, thickness, cv2.LINE_AA)
                #txt = "{:3n},{:3.2f},{:3n}".format(area, circ, approx)
                #cv2.putText(display_img, txt, (x,y+8), font, fontScale, color, thickness, cv2.LINE_AA)

            # Display all contours found
            if len(process.filter_contours_output) > 0:
                # fix the offset of the region of interest
                for contour in process.filter_contours_output:
                    # contour with new offset is created
                    contour += (startX, startY)
                cv2.drawContours(display_img, process.filter_contours_output, -1, (0,255,0), 1)
            # Show region of interest
            cv2.rectangle(display_img, (startX, startY), (endX, endY), (255,0,0), 2)
            cv2.imshow("Camera", display_img)
            
            switch_state = GPIO.input(switch_pin)

            if switch_state==0:
                break

            # Check if user want to quit
            try:
                if (cv2.waitKey(1) & 0xFF == ord('q')) or (cv2.getWindowProperty("Camera", 0) < 0): stop = True
            except: stop = True 

            ####
            # Move Arm depending on Ball Location
            ####
            ball_x=statistics.median(pos_x)
            ball_y=statistics.median(pos_y)
            # ball_x = pos_x # no filtering
            # ball_y = pos_y
            motor_x = x_gain*(width/2 -ball_x) + x_offset
            motor_z = z_gain*(height  -ball_y) + z_offset
            # print("{},{}".format(motor_x,motor_z))
            arm.goDirectlyTo(motor_x, motor_y, motor_z)


        ready_toThrow = False

        ####################################################
        # Detect Ball
        ####################################################
        cv2.startWindowThread()

        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
        picam2.start()

        process = WhiteBall()

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.25
        color = (0, 255, 0)
        thickness = 1

        # Main Loop
        stop = False

        while (not stop):
            img = picam2.capture_array()
            display_img = cv2.resize(img, ((int)(320), (int)(240)), 0, 0)
            
            process.process(img)

            if len(process.ball) > 0:
                x = process.ball[1] + process.startX
                y = process.ball[2] + process.startY
                hsv = cv2.cvtColor(display_img, cv2.COLOR_BGR2HSV)
                cv2.drawMarker(display_img, (x, y),  (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
                txt = "{},{},{}".format(hsv[y,x,0], hsv[y,x,1], hsv[y,x,2])
                cv2.putText(display_img, txt, (x,y), font, fontScale, color, thickness, cv2.LINE_AA)
                
                
            if len(process.filter_contours_output) > 0:
                # fix the offset of the region of interest
                x_offset, y_offset = process.startX, process.startY
                for contour in process.filter_contours_output:
                    # contour with new offset is created
                    contour += (x_offset, y_offset)
                cv2.drawContours(display_img, process.filter_contours_output, -1, (0,255,0), 1)

            x1=process.startX
            y1=process.startY
            x2=process.endX
            y2=process.endY
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (255,0,0), 2)
            cv2.imshow("Camera", display_img)

            
            if switch_state==0:
                ready_toThrow=True
                break
            try:
                if (cv2.waitKey(1) & 0xFF == ord('q')) or (cv2.getWindowProperty("Camera", 0) < 0): stop = True
            except: stop = True 
        ######################################
        # Move Arm depending on Ball Location
        ######################################
        break
        switch_state = GPIO.input(switch_pin)

    
    elif switch_state==0:
        
        ready_toThrow=True

        ####################################################
        # Attack
        ####################################################
        if ready_toThrow == True:
            #ready_toThrow = True
            arm.goDirectlyTo(0,150,70)
        
        start_state = GPIO.input(start_pin)

        if start_state and ready_toThrow:
            arm.closeGripper()
            arm.openGripper()
            arm.closeGripper()

# Clean up
cv2.destroyAllWindows()
GPIO.cleanup()
