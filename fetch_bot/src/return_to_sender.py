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

def return_ball(data: bool):
    if not data: return

    print("RETURN TO SENDER NOW")

    global pubUart

    y = m.sqrt(displaceX**2 + displaceY**2)

    msg = Drive()
    msg.rotation = adisplace
    msg.forward = 100 if y > 1 else 75 if y > 0.75 else 50
    pubUart.publish(msg)


def integrate(data: IMU):
    global displaceX, displaceY, velX, velY, adisplace

    print(f"AccelX: {data.accelx}, AccelY: {data.accely}, GyroZ: {data.gyroz}")

    displaceX += velX * dt + data.accelx * dt**2 / 2
    displaceY += velY * dt + data.accely * dt**2 / 2

    velX += data.accelx * dt
    velY += data.accely * dt

    adisplace += data.gyroz * dt

if __name__ == '__main__':
    rospy.init_node('return_to_sender')
    
    rospy.Subscriber("uart2return2sender", IMU, integrate)
    rospy.Subscriber("fetchBall2return2sender", Bool, return_ball)

    pubUart = rospy.Publisher("drive", Drive)
    rospy.spin()