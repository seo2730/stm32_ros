#!/usr/bin/env python
import roslib; roslib.load_manifest('stm32_ros_tutorial')
import rospy
from std_msgs.msg import UInt8

# Keyboard in Linux
import sys
import tty
import termios

def getKey():
    fd = sys.stdin.fileno()
    original_attributes = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
    return ch


def led(s):
    LedOnOff = s

    pub.publish(LedOnOff)
    #rate.sleep()

if __name__ == '__main__':
    rospy.init_node('stm32_ros_tutorial')

    pub = rospy.Publisher('led0',UInt8,queue_size=10)

    #rate = rospy.Rate(50)

    try:
        while(1):
            key=getKey()
            if key=='1':
                led(1)

            elif key=='0':
                led(0)

    except rospy.ROSInterruptException:
        rospy.loginfo("OFF")
