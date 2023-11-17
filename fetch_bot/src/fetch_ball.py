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
center_threshold = 0.05
sensitivity = 0.5
dist_factor = 16.1

frame_width = 1280 # in pixels
frame_height = 720 # in pixels
camera_fov_deg = 175.0  # in degrees

frame_width_center = frame_width / 2
center_width = frame_width * center_threshold
center_left = frame_width_center - center_width / 2
center_right = frame_width_center + center_width / 2

catchable_ball_size = 85 # something like this
catchable_ball_row = 660 # something like this

start = 0
count = 0

c_values = []
r_values = []
s_values = []

y_values = []
r_values = []

last_c, last_r, last_s = -1, -1, -1

frames_caught = 0
last_caught_frame_num = 0
frame_num = 0

# y is in m's and r is in radians
y, r = 0, 0

pubBD = None
pubDrive = None
pubRTS = None
pubArms = None

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
    global y, r
    global pubBD, pubDrive, pubRTS, pubArms


    frame_num += 1
    c, r, s = ball_pos_img.c, ball_pos_img.r, ball_pos_img.s
    # print(f"C: {c}, R: {r}, S: {s}")

    c_values.append(c)
    r_values.append(r)
    s_values.append(s)
    
    # if don't see ball
    if c == -1 and r == -1 and s == -1:
        # if last seen ball pos is near left or right edge then move robot that direction
        if last_c <= near_edge_threshold * frame_width:
            # turn left
            r = -1 * sensitivity * camera_fov_deg
        elif last_c >= (1 - near_edge_threshold) * frame_width:
            # turn right
            r = sensitivity * camera_fov_deg
        else:
            # drive forward
            y = 1
    elif isCatchable(c, r, s):
        if frames_caught == 0:
            # close arms
            pubArms.publish(True)

            frames_caught += 1
            last_caught_frame_num = frame_num
        elif frames_caught >= 1 and frames_caught < 5:
            if frame_num == last_caught_frame_num + 1:
                frames_caught += 1
                last_caught_frame_num = frame_num
            else:
                frames_caught = 0

                # open arms
                pubArms.publish(False)
        elif frames_caught >= 5:
            # stop ball detection
            pubBD.publish(True)

            # start return to sender
            pubRTS.publish(True)

            # turn off fetch_ball node
            rospy.sleep(0.5)
            rospy.signal_shutdown("time limit")
    else:
        last_c, last_r, last_s = c, r, s
        # Calculate horizontal angle based on the camera's FOV
        r = math.radians((c - (frame_width / 2)) / (frame_width / 2) * (camera_fov_deg / 2))

        # Calculate distance to the ball based on the apparent size
        y = dist_factor / s

    # send motor power percents to uart
    msg = Drive()
    msg.rotation = int(((r + math.pi) % (2 * math.pi) - math.pi) / math.pi * 100)
    msg.forward = 106 if y > 1 else 75 if y > 0.25 else 0
    pubDrive.publish(msg)

def fetch2(ball_pos_img: BallPosImg):
    global count
    c, r, s = ball_pos_img.c, ball_pos_img.r, ball_pos_img.s
    # print(f"C: {c}, R: {r}, S: {s}")
    r = math.radians((c - (frame_width / 2)) / (frame_width / 2) * (camera_fov_deg / 2))
    y = dist_factor / s
    msg = Drive()
    msg.rotation = int(((r + math.pi) % (2 * math.pi) - math.pi) / math.pi * 100)
    msg.forward = 100 if y > 1 else 75 if y > 0.75 else 50
    pubDrive.publish(msg)
    count += 1
    t = time.time() - start
    if t > 10:
        print(count/t)
        pubArms.publish(True)
        pubBD.publish(True)
        pubRTS.publish(True)
        rospy.sleep(0.5)
        rospy.signal_shutdown("time limit")

if __name__ == '__main__':
    rospy.init_node('fetch_ball')
    pubBD = rospy.Publisher("fetchBall2ballDetect", Bool, queue_size=10)
    pubRTS = rospy.Publisher("fetchBall2return2sender", Bool, queue_size=10)
    pubArms = rospy.Publisher("closeArmsUART", Bool, queue_size=10)
    pubDrive = rospy.Publisher("drive", Drive, queue_size=10)

    start = time.time()
    rospy.Subscriber("ballDetect2fetchBall", BallPosImg, fetch)
    rospy.spin()
