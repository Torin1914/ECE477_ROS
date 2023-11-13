#!/usr/bin/env python
import math
import numpy as np

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
    
def close_arms():
    pass

def return_to_sender():
    pass

def send_uart(data):
    pass

input_file = open("ball_pos_data.txt", 'r')
output_file = open("out_pos.txt", "w")

c_values = []
r_values = []
s_values = []

y_values = []
r_values = []

last_c, last_r, last_s = -1, -1, -1

frames_caught = 0

# -1 y means continue with same lateral velocity
# x and y are in m's and r is in radians
y, r = -1, 0

for line in input_file:
    values = line.strip().split(', ')
    c, r, s = map(int, values)

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
            continue
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
        # close arms and verify ball has been caught
        close_arms()
        frames_caught += 1
        if frames_caught >= 3:
            # then start return to sender and stop ball detection
            return_to_sender()
    else:
        # Calculate horizontal angle based on the camera's FOV
        r = math.radians((c - (frame_width / 2)) / (frame_width / 2) * (camera_fov_deg / 2))

        # Calculate distance to the ball based on the apparent size
        y = dist_factor / s

    y_values.append(y)
    r_values.append(r)
    output_file.write(f"{y}, {r}\n")
    send_uart(y, r)




# return to sender should be easy. prob can find online a library that will take all the IMU
# data and turn it into positions. then just send that initial pos to MCU


input_file.close()
output_file.close()
