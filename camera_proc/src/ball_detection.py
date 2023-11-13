#!/usr/bin/env python
import cv2 as cv 
import numpy as np
import time
import rospy


if __name__ == '__main__':
    rospy.init_node("ball_detection")
    pub = rospy.Publisher("ballDetect2fetchBall", )
    
    pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)120/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink sync=false"
    )

    video = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
    if not video.isOpened():
        print("Camera not opened")

    # pink thresholds
    lower_pink = np.array([140, 100, 50])   # Lower bound for pink color in HSV
    upper_pink = np.array([170, 255, 255])  # Upper bound for pink color in HSV

    times = []
    begin = time.time()


    while time.time()-begin < 10:
        ret, frame = video.read()
        if not ret: break

        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        pink_mask = cv.inRange(frame_hsv, lower_pink, upper_pink)

        blurFrame = cv.GaussianBlur(pink_mask, (17,17), 0)
        circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1.4, 5000,
                                param1=170, param2=20, minRadius=0, maxRadius=100)
        
        if circles is not None:
            # column, row, size (radius of ball in pixels)
            c, r, s = circles[0,0,0], circles[0,0,1], circles[0,0,2]
            # print(f"{c}, {r}, {s}\n")
        #     file.write(f"{c}, {r}, {s}\n")
        # else:
        #     file.write(f"-1, -1, -1\n")

    video.release()
