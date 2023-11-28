#!/usr/bin/env python3
import rospy
import math as m
from fetch_bot.msg import IMU
from fetch_bot.msg import Drive
from std_msgs.msg import Bool

displaceX, displaceY = 0, 0
velX, velY = 0, 0
adisplace = 0
oldDisX, oldDisY = 0, 0  # Declare oldDisX and oldDisY as global variables
oldAdis = 0

dt = 1.0/60

flag = False

pubUart = None

def return_ball(data: bool):
    global displaceX, displaceY, adisplace, oldDisX, oldDisY, pubUart, oldAdis, flag
    
    msg = Drive()
    
    # want to calc angle to rotate, then to rotate that angle calc rough
    # estimate of speed and a wait time until stop rotation
    # at that point ball detection should pick up the endzone ball

    # clockwise rotation is positive

    # a positve adisplace could actually be a negative oh god

    # first mod by 2 pi to get angle on unit circle
    if adisplace >= 0:
        adisplace %= m.pi * 2
    else:
        adisplace *= -1
        adisplace %= m.pi * 2
        adisplace *= -1

    # normalizing angle to usable first is necessary

    pubUart.publish(msg)

    msg.rotation = (m.pi/2) - adisplace

def integrate(data: IMU):
    global displaceX, displaceY, velX, velY, adisplace, oldDisX, oldDisY, pubUart

    # print(f"AccelX: {data.accelx}, AccelY: {data.accely}, GyroZ: {data.gyroz}")

    displaceX += velX * dt + data.accelx * dt**2 / 2
    displaceY += velY * dt + data.accely * dt**2 / 2

    velX += data.accelx * dt
    velY += data.accely * dt

    adisplace += data.gyroz * dt
    
    if flag:
        msg = Drive()

        




        
        if msg.rotation != adisplace:
            msg.rotation = 50
        
        pubUart.publish(msg)


if __name__ == '__main__':
    rospy.init_node("return_to_sender")

    rospy.Subscriber("fetchBall2return2sender", Bool, return_ball)
    rospy.Subscriber("uart2return2sender", IMU, integrate)

    pubUart = rospy.Publisher("driveFinal", Drive, queue_size=10)
    rospy.spin()
    