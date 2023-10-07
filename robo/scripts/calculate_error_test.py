#!/usr/bin/env python2.7

## This script is used to calculate the error between the mocap and the robot pose.
## It was adapted to compute the average error between the mocap and the robot pose.

import rospy
import rosbag
import sys
import argparse
import numpy
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TransformHandler():

    def __init__(self, gt_frame, est_frame, new_frame, max_time_between=0.01):
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.new_frame = new_frame
        self.frames = [gt_frame, est_frame, new_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.__tf_listener = TransformListener(self.tf_buffer)

        #self.warn_timer = rospy.Timer(rospy.Duration(5), self.__warn_timer_cb)

    def __warn_timer_cb(self, evt):

        available_frames = self.tf_buffer.all_frames_as_string()
        avail = True
        for frame in self.frames:
            if frame not in available_frames:
                rospy.logwarn('Frame {} has not been seen yet'.format(frame))
                avail = False
        if avail:
            self.warn_timer.shutdown()

    def get_transform(self, fixed_frame, target_frame):
        # caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0))
    
    

def get_errors(transform):
    tr = transform.transform.translation
    return numpy.linalg.norm([tr.x, tr.y])

def callback(data):
    # Access the covariance matrix
    covariance_matrix = data.pose.covariance

    # Extract the relevant variance element for the dimension
    var_i = covariance_matrix[0]  # Diagonal element for dimension i

    # Calculate the standard deviation (uncertainty) for the dimension
    uncertainty_i = np.sqrt(var_i)

    print('Uncertainty for x: ' + str(uncertainty_i))

    transform = TransformStamped()
    transform.header.frame_id = "base_scan"  # Original frame
    transform.child_frame_id = "new_frame"   # New frame
    transform.transform.translation.x = uncertainty_i  # Adjust the translation values based on uncertainty
    transform.transform.translation.y = 0.0  # You can adjust the y and z translations as needed
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0  # No rotation

    new_frame = args.new_frame
    gt_frame = args.gt_frame

    t = handler.get_transform(gt_frame, new_frame)

parser = argparse.ArgumentParser()
parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')
parser.add_argument('--new_frame', help='The child frame of the estimation transform', default='new_frame')

args = parser.parse_args()

gt_frame = args.gt_frame
est_frame = args.est_frame
new_frame = args.new_frame

rospy.init_node('evaluation_node')

if rospy.rostime.is_wallclock():
    rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
    sys.exit(1)

rospy.loginfo('Waiting for clock')
rospy.sleep(0.00001)

handler = TransformHandler(gt_frame, est_frame, new_frame, max_time_between=20) # 500ms

rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')
sleeper = rospy.Rate(1000)

rospy.Subscriber('/odometry/filtered', Odometry, callback)
rospy.spin()
