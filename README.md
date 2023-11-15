# purdueECE477

mtx: [[ 571.75864487    0.          630.67238944]
 [   0.          571.38160776  356.21218923]
 [   0.            0.            1.        ]]

 dist: [[-0.28155159  0.09339524 -0.00264697 -0.00076021  0.06938456]]

 ncm: [[ 533.23010254    0.          631.92690463]
 [   0.          499.39093018  353.75964042]
 [   0.            0.            1.        ]]

roi: (169, 139, 873, 432)

total error: 0.002428373180923953

run this cmd every time open new terminal when working with ROS
source devel/setup.bash


ROS nodes:
1. ball_detection
    sends ball_pos_img.msg to fetch_ball            done
    receives stop bool from fetch_ball              done
2. fetch_ball
    receives ball_pos_img.msg from ball_detection   done
    sends stop bool to ball_detection               done
    sends y to driveDis                             done not tested
    sends r to driveRot                             done not tested
    sends start bool to return_to_sender            done not tested
    sends close_arms bool to uart
3. driveDis
    receives y from fetch_ball
    sends forward_power to uart
    receives imu.msg from uart
4. driveRot
    received r from fetch_ball
    sends rotation_power to uart
    receives imu.msg from uart
5. uart
    receives forward_power from driveDis
    receives rotation_power from driveRot
    receives closeArms from fetchBall
    sends imu.msg to driveDis
    sends imu.msg to driveRot
6. return_to_sender
    receives start bool from fetch_ball             done not tested
    receives imu.msg from uart                      done not tested
    sends y to driveDis                             done not tested
    sends r to driveRot                             done not tested

ROS msgs:
    ball_pos_img.msg:
        float32 c
        float32 r
        float32 s
    node_control.msg:
        bool run
    ball_pos.msg:
        float32 y
        float32 r
    bot_control.msg:
        int32 forward -100 to 100 in percent power
        int32 rotation -100 to 100 in percent power
        bool close_arms
    imu.msg:
        float32 accel_x
        float32 accel_y
        float32 accel_z
        float32 gyro_x
        float32 gyro_y
        float32 gyro_z
