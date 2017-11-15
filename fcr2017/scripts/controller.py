#!/usr/bin/env python
# pylint: disable=C0103

import tty
import sys
from termios import tcflush, TCIFLUSH, tcsetattr, tcgetattr, TCSADRAIN
from std_msgs.msg import Char
import rospy

INSTRUCTIONS = """
      ^
   Q  W  E
<  A     D  >

S - Stop
"""
CTRL_C = 3
valid_keys = ['a', 'q', 'w', 'e', 'd', 'r', 's']


if __name__ == "__main__":
    pub = rospy.Publisher('controller', Char, queue_size=1)
    rospy.init_node('controller', anonymous=True)
    settings = tcgetattr(sys.stdin)
    msg = Char()

    print INSTRUCTIONS
    tty.setraw(sys.stdin.fileno())

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

        # Clear the input buffer
        tcflush(sys.stdin, TCIFLUSH)

        key = sys.stdin.read(1).lower()

        # Verify if Ctrl C was pressed
        if ord(key) == CTRL_C:
            break

        if key in valid_keys:
            pub.publish(ord(key))
        else:
            pub.publish(0)

    tcsetattr(sys.stdin, TCSADRAIN, settings)
