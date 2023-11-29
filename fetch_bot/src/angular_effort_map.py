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
    print(rows)
    with open("angular_to_velocity.csv", 'w') as csvfile: 
        csvwriter = csv.writer(csvfile)  
        for row in rows:
            csvwriter.writerow(row) 
    sleep(0.1)

def write_angular(data: IMU):
    global drive
    global count
    global rows
    row = [drive.rotation, data.gyroz]
    if count > 50:
        count = 0
        if drive.rotation > 0:
            drive.rotation = drive.rotation - 10
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

