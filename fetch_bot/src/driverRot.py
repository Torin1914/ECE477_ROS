#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import math 



def callback(data):
    global fetchTheta 
    rospy.loginfo("Received: %f", data.data)
    fetchTheta = data.data
    
    #normalize fetch_bll received R value
    nFetchTheta = (data.data + math.pi) / (math.pi + math.pi ) * (100 + 100)
    #nFetchTheta = (((data.data + math.pi) * 200 ) / (2 * math.pi)) - 180 
    print("NORMALIZE: ", nFetchTheta) 
    

def listener():
    rospy.init_node('rotationLoop')
    rospy.Subscriber('fetchBall2driveRotation', Float32, callback)
    rospy.spin()




if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass



'''

fetchTheta = 0 
nFetchTheta = (((fetchTheta + math.pi) * 200 ) / (2 * math.pi)) - 180 
print(nFetchTheta) 


imuTheta = 185
nImuTheta = (imuTheta/360) * 200 - 100 
print(nImuTheta)


while True:
    deltaTheta = nImuTheta - nFetchTheta  
    if abs(deltaTheta) <= 1:
        print("similar enough")
    else:
        print("difference: ", deltaTheta)

'''


