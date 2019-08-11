#!/usr/bin/env python

from __future__ import print_function

import roslib; 
# roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

def callback(data):
    print("Have smt")
    print(data)
    rospy.loginfo(rospy.get_name())
    rospy.loginfo(data)

if __name__ == '__main__':
    listener()
