import rospy
from fetch_bot.msg import IMU

accelX_values = []
accelY_values = []
accelZ_values = []

gyroX_values = []
gyroY_values = []
gyroZ_values = []

pubDriveDis = None
pubDriveRot = None

def return_ball(data: bool):
    if not data: return

    global pubDriveRot, pubDriveDis

    # do some wack math to calc y and r

    y, r = 0, 0

    pubDriveDis.publish(y)
    pubDriveRot.publish(r)



def accumulate(data: IMU):
    global accelY_values, accelX_values, accelZ_values
    global gyroX_values, gyroY_values, gyroZ_values

    accelX_values.append(data.accel_x)
    accelY_values.append(data.accel_y)
    accelZ_values.append(data.accel_z)

    gyroX_values.append(data.gyro_x)
    gyroY_values.append(data.gyro_y)
    gyroZ_values.append(data.gyro_z)

if __name__ == '__main__':
    rospy.init_node('return_to_sender')
    
    rospy.Subscriber("uart2return2sender", IMU, accumulate)
    rospy.Subscriber("fetchBall2return2sender", bool, return_ball)

    pubDriveDis = rospy.Publisher("return2sender2driveDisplacement", float)
    pubDriveRot = rospy.Publisher("return2sender2driveRotation", float)
    rospy.spin()