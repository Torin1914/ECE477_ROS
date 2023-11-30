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
csvwriter = None
fd = None


def handler(signum, frame): #called when ctrl-C interrupt is detected
    global fd
    global rows
    print('Ctrl-C detected')
    fd.close()
    sleep(0.1)

def write_angular(data: IMU):
    global drive
    global count
    global rows
    global csvwriter
    row = [drive.rotation, data.gyroz]
    if count > 50:
        count = 0
        if drive.rotation > 0:
            drive.rotation = drive.rotation - 2
        pubUart.publish(drive)
    else:
        count = count + 1
    print("getting here")
    csvwriter.writerow(row)
if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    rospy.init_node("angular_map")
    sub = rospy.Subscriber("imu", IMU, write_angular)
    pubUart = rospy.Publisher("drive", Drive, queue_size=10)
    pubUart.publish(drive)

    fd = open("data.csv", 'w')
    csvwriter = csv.writer(fd)

    csvwriter.writerow(rows)
    rospy.spin()

