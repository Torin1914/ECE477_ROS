#!/usr/bin/env python3
import math
import numpy as np
import rospy
import time
from fetch_bot.msg import BallPosImg
from fetch_bot.msg import Drive
from std_msgs.msg import Bool
from std_msgs.msg import Float32

near_edge_threshold = 0.1
center_threshold = 0.1
sensitivity = 0.8
dist_factor = 16.1

frame_width = 1280.0 # in pixels
frame_height = 720.0 # in pixels
camera_fov_deg = 175.0  # in degrees

frame_width_center = frame_width / 2
center_width = frame_width * center_threshold
center_left = frame_width_center - center_width / 2
center_right = frame_width_center + center_width / 2

catchable_ball_size = 110.0 # something like this
catchable_ball_row = 400.0 # something like this

start = 0
count = 0

c_values = []
r_values = []
s_values = []

last_c, last_r, last_s = -1, -1, -1

frames_caught = 0
last_caught_frame_num = 0
frame_num = 0

# dist is in m's and rot is in radians
dist, rot = 0, 0

pubBD = None
pubDrive = None
pubRTS = None
pubArms = None

flag = False

def isInCenter(c):
    if c > center_left and c < center_right:
        return True
    else:
        return False
    
def isCatchable(c, r, s):
    if s > catchable_ball_size and isInCenter(c) and r > catchable_ball_row:
        return True
    else:
        return False

def fetch(ball_pos_img: BallPosImg):
    global c_values, r_values, s_values
    global last_c, last_r, last_s

    global frames_caught, frame_num, last_caught_frame_num
    global dist, rot
    global pubBD, pubDrive, pubRTS, pubArms
    global center_threshold, catchable_ball_row
    global flag


    frame_num += 1
    c, r, s = ball_pos_img.c, ball_pos_img.r, ball_pos_img.s
    print(f"C: {c}, R: {r}, S: {s}")

    c_values.append(c)
    r_values.append(r)
    s_values.append(s)
    
    # if don't see ball
    if c == -1 and r == -1 and s == -1:
        # if last seen ball pos is near left or right edge then move robot that direction
        if last_c <= near_edge_threshold * frame_width:
            # turn left
            rot = -1 * sensitivity
        elif last_c >= (1 - near_edge_threshold) * frame_width:
            # turn right
            rot = sensitivity
        else:
            # drive forward
            dist = 1
    elif not flag and isCatchable(c, r, s):
        print("***********CATCHABLE************")
        if frames_caught == 0:
            # close arms
            pubArms.publish(True)

            # increase thresholds
            center_threshold = 0.5
            catchable_ball_row = 200

            frames_caught += 1
            last_caught_frame_num = frame_num
        elif frames_caught >= 1 and frames_caught < 3:
            if frame_num == last_caught_frame_num + 1:
                frames_caught += 1
                last_caught_frame_num = frame_num
            else:
                frames_caught = 0

                # decrease thresholds
                center_threshold = 0.1
                catchable_ball_row = 400

                # open arms
                pubArms.publish(False)
        elif frames_caught >= 3:
            # start detecting green ball
            pubBD.publish(True)

            # toggle flag
            flag = True

            '''
            # stop ball detection
            pubBD.publish(True)
            
            # start return to sender
            pubRTS.publish(True)

            # turn off fetch_ball node
            rospy.sleep(0.5)
            rospy.signal_shutdown("time limit")
            '''
    elif flag and s >= 100 and isInCenter(c):
        # stop and release ball
        msg = Drive()
        msg.forward = 0
        msg.rotation = 0
        pubDrive.publish(msg)
        pubArms.publish(False)
        rospy.sleep(0.5)
        rospy.signal_shutdown("time limit")
    else:
        last_c, last_r, last_s = c, r, s
        # Calculate horizontal angle based on the camera's FOV
        rot = math.radians((c - (frame_width / 2)) / (frame_width / 2) * (camera_fov_deg / 2))

        # Calculate distance to the ball based on the apparent size
        dist = dist_factor / s

    # print(f"rot: {rot}, dist: {dist}")

    if frames_caught == 0:
        # send motor power percents to uart
        msg = Drive()

        # rotation
        if rot < -math.pi/4 and dist > 1:
            msg.rotation = -100
        elif rot > math.pi/4 and dist > 1:
            msg.rotation = 100
        else:
            msg.rotation = int((rot / (math.pi/2)) * 100)

        # forward
        msg.forward = 106 if dist > 1 else 75 if dist > 0.25 else 40 if dist > 0.1 else 0

        pubDrive.publish(msg)

if __name__ == '__main__':
    rospy.init_node('fetch_ball')
    pubBD = rospy.Publisher("fetchBall2ballDetect", Bool, queue_size=10)
    pubRTS = rospy.Publisher("fetchBall2return2sender", Bool, queue_size=10)
    pubArms = rospy.Publisher("closeArmsUART", Bool, queue_size=10)
    pubDrive = rospy.Publisher("drive", Drive, queue_size=10)

    start = time.time()
    rospy.Subscriber("ballDetect2fetchBall", BallPosImg, fetch)
    rospy.spin()
