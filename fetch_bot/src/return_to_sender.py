import rospy
import math as m
from fetch_bot.msg import IMU
from std_msgs.msg import Bool
from std_msgs.msg import Float32

displaceX, displaceY = 0, 0
velX, velY = 0, 0
adisplace = 0

dt = 1.0/60

pubDriveDis = None
pubDriveRot = None

def return_ball(data: bool):
    if not data: return

    global pubDriveRot, pubDriveDis

    y = m.sqrt(displaceX**2 + displaceY**2)

    pubDriveDis.publish(y)
    pubDriveRot.publish(adisplace)


def integrate(data: IMU):
    global displaceX, displaceY, velX, velY, adisplace

    displaceX += velX * dt + data.accelx * dt**2 / 2
    displaceY += velY * dt + data.accely * dt**2 / 2

    velX += data.accelx * dt
    velY += data.accely * dt

    adisplace += data.gyroz * dt

if __name__ == '__main__':
    rospy.init_node('return_to_sender')
    
    rospy.Subscriber("uart2return2sender", IMU, integrate)
    rospy.Subscriber("fetchBall2return2sender", Bool, return_ball)

    pubDriveDis = rospy.Publisher("return2sender2driveDisplacement", Float32)
    pubDriveRot = rospy.Publisher("return2sender2driveRotation", Float32)
    rospy.spin()