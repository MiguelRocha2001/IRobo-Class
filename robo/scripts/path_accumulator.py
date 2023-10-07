#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

class CombinedPathAccumulator:
    def __init__(self):
        # Initialize node
        rospy.init_node('combined_path_accumulator')

        # Create a buffer and listener for tf
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers for the accumulated paths
        self.path_pub_mocap_laser = rospy.Publisher('/mocap_laser_path_in_mocap', nav_msgs.msg.Path, queue_size=10)
        self.path_pub_odom = rospy.Publisher('/odom_path', nav_msgs.msg.Path, queue_size=10)  # Added publisher
        
        # Path messages to store accumulated paths
        self.path_mocap_laser = nav_msgs.msg.Path()
        self.path_mocap_laser.header.frame_id = "mocap"

        # Subscriber for odometry data
        self.odom_sub = rospy.Subscriber('/odometry/filtered', nav_msgs.msg.Odometry, self.odom_callback)

        # Path message to store path from odometry data
        self.path_odom = nav_msgs.msg.Path()
        self.path_odom.header.frame_id = "odom"

    def odom_callback(self, msg):
        # Get the pose from the Odometry message and append it to the path
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header = msg.header  # using the header from the Odometry message to keep the timestamp and frame_id
        pose_stamped.pose = msg.pose.pose  # using the pose from the Odometry message
        self.path_odom.poses.append(pose_stamped)

        # Publish the path
        self.path_pub_odom.publish(self.path_odom)

    def accumulate_paths(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            try:
                # Dynamic transform from mocap to mocap_laser_link
                trans_mocap_to_laser = self.tf_buffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time(0))
                pose_mocap_laser = geometry_msgs.msg.PoseStamped()
                pose_mocap_laser.header.stamp = rospy.Time.now()
                pose_mocap_laser.header.frame_id = "mocap"
                pose_mocap_laser.pose.position = trans_mocap_to_laser.transform.translation
                pose_mocap_laser.pose.orientation = trans_mocap_to_laser.transform.rotation
                self.path_mocap_laser.poses.append(pose_mocap_laser)
                self.path_pub_mocap_laser.publish(self.path_mocap_laser)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            rate.sleep()

if __name__ == '__main__':
    pa = CombinedPathAccumulator()
    pa.accumulate_paths()