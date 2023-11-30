#!/usr/bin/env python3
import rospy
import math as m
from fetch_bot.msg import IMU
from fetch_bot.msg import Drive
from std_msgs.msg import Bool

displaceX, displaceY = 0, 0
velX, velY = 0, 0
adisplace = 0

dt = 1.0/60

pubUart = None

def aEffort2aVel(aEffort: int):
    return 0.11 * aEffort - 1.22

def final_rotation(data: bool):
    global adisplace
    twoPi = m.pi * 2
    angle2rotate = 0

    msg = Drive()
    msg.forward = 0

    adisplace = m.radians(adisplace)
    adisplace %= twoPi

    if adisplace > m.pi:
        angle2rotate = twoPi - adisplace
    else:
        angle2rotate = adisplace * -1

    # arbitrary rotation effort
    msg.rotation = 50
    pubUart.publish(msg)

    # use Zach's fancy function to figure out how long to spin based on rotation effort
    aVel = aEffort2aVel(msg.rotation) * m.pi / 180
    time = abs(angle2rotate / aVel)
    rospy.sleep(time)
    msg.rotation = 0
    pubUart.publish(msg)
    rospy.sleep(4)
    rospy.signal_shutdown("done")

def integrate(data: IMU):
    global displaceX, displaceY, velX, velY, adisplace

    # print(f"AccelX: {data.accelx}, AccelY: {data.accely}, GyroZ: {data.gyroz}")

    displaceX += velX * dt + data.accelx * dt**2 / 2
    displaceY += velY * dt + data.accely * dt**2 / 2

    velX += data.accelx * dt
    velY += data.accely * dt

    adisplace += data.gyroz * dt

if __name__ == '__main__':
    rospy.init_node("return_to_sender")

    rospy.Subscriber("final_rotation", Bool, final_rotation)
    rospy.Subscriber("imu", IMU, integrate)

    pubUart = rospy.Publisher("driveFinal", Drive, queue_size=10)
    rospy.spin()
    