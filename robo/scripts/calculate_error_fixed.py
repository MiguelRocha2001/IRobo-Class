#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg, str):
    # Extract position data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    rospy.loginfo(str + ":" +"Position: x=%.3f, y=%.3f, z=%.3f", x, y, z)

def listener():
    # Initialize the node with a name
    rospy.init_node('odometry_listener', anonymous=True)
    
    # Subscribe to the /odometry/filtered topic
    rospy.Subscriber('/odometry/filtered', Odometry, callback, "Filtered: ")
    rospy.Subscriber('/odom', Odometry, callback, "Odom: ")
    
    # Keep the node running until it's shut down
    rospy.spin()

if __name__ == '__main__':
    listener()