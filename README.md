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


ROS nodes:  remember to chmod +x
1. ball_detection
    sends BallPosImg.msg to fetch_ball              done
    receives stop bool from fetch_ball              done
2. fetch_ball
    receives BallPosImg.msg from ball_detection     done
    sends stop bool to ball_detection               done
    sends Drive.msg to uart                         done
    sends start bool to return_to_sender            done not tested
    sends close_arms bool to uart                   done not tested
3. uart
    receives Drive.msg from fetch_ball              done
    receives closeArms from fetchBall               done not tested
    sends IMU.msg to return_to_sender               done
4. return_to_sender
    receives start bool from fetch_ball             done not tested
    receives imu.msg from uart                      done
    sends Drive.msg to uart                         done not tested

to run uart: i think it was
sudo usermod -aG root $USER
this should persist when restarting but ya