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

dt = 1.0/60

flag = False

pubUart = None

def return_ball(data: bool):
    global displaceX, displaceY, adisplace, oldDisX, oldDisY, pubUart, oldAdis, rotation
    msg = Drive()
    rotation = (m.pi/2) - adisplace
    rotation = (m.degrees(rotation) % 360) - 180
    rotation *= 100 / 180
    
    
    oldDisY = displaceY
    oldDisX = displaceX
    oldAdis = adisplace
    displaceY = 0 
    displaceX = 0
    adisplace = 0 
    
    flag = True

    pubUart.publish(msg)

def integrate(data: IMU):
    global displaceX, displaceY, velX, velY, adisplace, oldDisX, oldDisY, pubUart

    # print(f"AccelX: {data.accelx}, AccelY: {data.accely}, GyroZ: {data.gyroz}")

    displaceX += velX * dt + data.accelx * dt**2 / 2 #once got signal to return to sender change this to 0
    displaceY += velY * dt + data.accely * dt**2 / 2 #once got signal to return to sender change this to 0

    velX += data.accelx * dt #once got signal to return to sender change this to 0
    velY += data.accely * dt #once got signal to return to sender change this to 0

    adisplace += data.gyroz * dt #once got signal to return to sender change this to 0
    #-pi/4 to pi/4
    if flag:
        print("RETURN TO USER STARTED")
        msg = Drive()
        if rotation != adisplace:
            msg.rotation = 50
        else:
            pass
        
        if (0.9 * displaceX) < oldDisX < (1.1 * displaceX):
            pass
        elif(0.9 * displaceY) < oldDisY < (1.1 * displaceY):
            pass
        if displaceY < oldDisY or displaceX < oldDisX:
            msg.forward = 75
        elif displaceY > oldDisY or displaceX > oldDisX:
            msg.forward = -75
        pubUart.publish(msg)


if __name__ == '__main__':
    rospy.init_node("return_to_sender")

    rospy.Subscriber("fetchBall2return2sender", Bool, return_ball)
    rospy.Subscriber("uart2return2sender", IMU, integrate)

    pubUart = rospy.Publisher("driveFinal", Drive, queue_size=10)
    rospy.spin()
    