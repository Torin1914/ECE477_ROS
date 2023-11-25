#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from fetch_bot.msg import Drive
from std_msgs.msg import Bool

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

global pub
global arms_pub
actuated = False

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def updater(key):
    global pub
    global arms_pub
    global actuated
    drive = Drive()
    if key == "w":
        drive.forward = 100
    elif key == "s":
        drive.forward = -100
    else:
        drive.forward = 0
    
    if key == "a":
        drive.rotation = -100
    elif key == "d":
        drive.rotation = 100
    else:
        drive.rotation = 0
    
    if key == " ":
        if actuated == False:
            actuated = True
        else:
            actuated = False

        
        arms_pub.publish(actuated)

    if key == "c" or key == "x":
        rospy.signal_shutdown()

    pub.publish(drive)

def talker():
    global pub
    global arms_pub

    settings = saveTerminalSettings()
    key_timeout = rospy.get_param("~key_timeout", 0.1)

    pub = rospy.Publisher('drive', Drive, queue_size=10)
    arms_pub = rospy.Publisher('closeArmsUART', Bool, queue_size=10)
    rospy.init_node('gamepad', anonymous=True)

    while not rospy.is_shutdown():
        key = getKey(settings, key_timeout)
        updater(key)

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
         pass
