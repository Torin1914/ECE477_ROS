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
    
    flag = True
    oldDisY = displaceY
    oldDisX = displaceX
    oldAdis = adisplace
    displaceY = 0 
    displaceX = 0
    adisplace = 0 

    msg = Drive()
    msg.rotation = 50 if adisplace > 0 else -50 if adisplace < 0 else 0
    msg.forward = 0
    pubUart.publish(msg)

    # msg.rotation = (m.pi/2) - adisplace
    # msg.rotation = (m.degrees(rotation) % 360) - 180
    # msg.rotation *= 100 / 180

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
    