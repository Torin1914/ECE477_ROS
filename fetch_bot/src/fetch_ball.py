#!/usr/bin/env python3
import math
import numpy as np
import rospy
import time
from fetch_bot.msg import BallPosImg
from fetch_bot.msg import NodeControl

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

# -1 y means continue with same lateral velocity
# y is in m's and r is in radians
y, r = -1, 0

pubBD = None
pubDriveDis = None
pubDriveRot = None
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

    global frames_caught
    global y, r
    global pubBD, pubDriveDis, pubDriveRot, pubRTS, pubArms

    c, r, s = ball_pos_img.c, ball_pos_img.r, ball_pos_img.s

    last_c, last_r, last_s = c, r, s
    c_values.append(c)
    r_values.append(r)
    s_values.append(s)
    
    # if don't see ball
    if c == -1 and r == -1 and s == -1:
        # if didn't see ball on last frame
        if last_c == -1 and last_r == -1 and last_s == -1:
            # wait until see ball
            # this prob isn't right because ball could easily flicker out two frames in a row
            pass
        else:
            # if last ball pos is near left or right edge then move robot that direction
            if last_c <= near_edge_threshold * frame_width:
                # turn left
                r = -1 * sensitivity * camera_fov_deg
            elif last_c >= (1 - near_edge_threshold) * frame_width:
                # turn right
                r = sensitivity * camera_fov_deg
            else:
                # rotate towards last seen location of ball
                r = last_r
    elif isCatchable(c, r, s):
        # close arms
        pubArms.publish(True)

        # verify ball has been caught
        # TODO this doesn't make much sense or corrects for if we didn't actually catch the ball
        frames_caught += 1
        if frames_caught >= 3:
            # stop ball detection
            pubBD.publish(False)

            # start return to sender
            pubRTS.publish(True)

            # TODO end sending data to Drive
    else:
        # Calculate horizontal angle based on the camera's FOV
        r = math.radians((c - (frame_width / 2)) / (frame_width / 2) * (camera_fov_deg / 2))

        # Calculate distance to the ball based on the apparent size
        y = dist_factor / s

    # send ball position to both drive nodes
    pubDriveDis.publish(y)
    pubDriveRot.publish(r)
    print(f"Y: {y}, R: {r}")

def fetch2(ball_pos_img: BallPosImg):
    global count
    c, r, s = ball_pos_img.c, ball_pos_img.r, ball_pos_img.s
    print(f"C: {c}, R: {r}, S: {s}")
    count += 1
    t = time.time() - start
    if t > 10:
        print(count/t)
        msg_send = NodeControl()
        msg_send.run = False
        pubBD.publish(msg_send)
        rospy.signal_shutdown("time limit")

if __name__ == '__main__':
    rospy.init_node('fetch_ball')
    pubBD = rospy.Publisher("fetchBall2ballDetect", bool)
    pubRTS = rospy.Publisher("fetchBall2return2sender", bool)
    pubDriveDis = rospy.Publisher("fetchBall2driveDisplacement", float)
    pubDriveRot = rospy.Publisher("fetchBall2driveRotation", float)
    pubArms = rospy.Publisher("closeArmsUART", bool)

    start = time.time()
    rospy.Subscriber("ballDetect2fetchBall", BallPosImg, fetch)
    rospy.spin()
