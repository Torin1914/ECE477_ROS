#!/usr/bin/env python3
import cv2 as cv 
import numpy as np
import rospy
from fetch_bot.msg import BallPosImg
from std_msgs.msg import Bool

video = None

def shutdown(data: bool):
    global video
    if data:
        video.release()
        print("************ball detection stopped")
        rospy.signal_shutdown("STOP BALL DETECTION")

if __name__ == '__main__':
    rospy.init_node("ball_detection")
    pub = rospy.Publisher("ballDetect2fetchBall", BallPosImg, queue_size=10)
    rospy.Subscriber("fetchBall2ballDetect", Bool, shutdown, queue_size=10)

    msg_send = BallPosImg()
    
    pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)120/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true"
    )
    video = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
    
    # out_send = cv.VideoWriter('appsrc ! videoconvert ! omxh264enc control-rate=constant bitrate=5000000 iframeinterval=15 ! h264parse ! rtph264pay pt=96 config-interval=10 ! udpsink host=192.168.0.100 port=5000 sync=false', cv.CAP_GSTREAMER, 0, 30, (1280, 720))

    if not video.isOpened():
        print("Camera not opened")

    # pink thresholds
    lower_pink = np.array([140, 100, 50])   # Lower bound for pink color in HSV
    upper_pink = np.array([170, 255, 255])  # Upper bound for pink color in HSV

    while not rospy.is_shutdown():
        ret, frame = video.read()
        if not ret: break

        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        pink_mask = cv.inRange(frame_hsv, lower_pink, upper_pink)

        blurFrame = cv.GaussianBlur(pink_mask, (17,17), 0)
        circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1.4, 5000,
                                param1=170, param2=20, minRadius=0, maxRadius=200)
        
        if circles is not None:
            # column, row, size (radius of ball in pixels)
            msg_send.c, msg_send.r, msg_send.s = circles[0,0,0], circles[0,0,1], circles[0,0,2]
            cv.circle(frame, (circles[0,0,0], circles[0,0,1]), circles[0,0,2], (0,255,0), 2)
        # cv.imshow("", frame)
        # if cv.waitKey(1) & 0xFF == ord('q'): break

        else:
            msg_send.c, msg_send.r, msg_send.s = -1, -1, -1
        pub.publish(msg_send)
        # out_send.write(frame)


    video.release()
    cv.destroyAllWindows()
