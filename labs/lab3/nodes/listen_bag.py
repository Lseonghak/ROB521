#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo("Received odometry message:\n%s", data)

def listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/odom_est", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()