#!/usr/bin/env python3

import rospy
from fetch_bot.msg import IMU
from fetch_bot.msg import Drive
from time import sleep
import csv
import signal

rows = [["angular effort (%)", "Angular Velocity (rads/s)"]]
count = 0
drive = Drive()
drive.forward = 0
drive.rotation = 100


def handler(signum, frame): #called when ctrl-C interrupt is detected
    global rows
    print('Ctrl-C detected')
    with open("angular_to_velocity.csv", 'w') as csvfile: 
        csvwriter = csv.writer(csvfile)  
        csvwriter.writerows(rows) 
        sleep(0.1)
        csvfile.close() # when you're done.

def write_angular(data):
    global drive
    global count
    global rows
    row = [drive.rotate, data.gyroz]
    if count > 50:
        count = 0
        if drive.rotate > 0:
            drive.rotate = drive.rotate - 10
        pubUart.publish(drive)
    else:
        count = count + 1
    rows.append(row)
if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    rospy.init_node("angular_map")
    sub = rospy.Subscriber("imu", IMU, write_angular)
    pubUart = rospy.Publisher("drive", Drive, queue_size=10)
    pubUart.publish(drive)

    rospy.spin()

