#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import math
import numpy as np
import matplotlib.pyplot as plt

# Global variables
total_squared_error = 0.0
total_error = 0.0
num_of_errors = 0
rmse_list = []
frame_list = []
std_error_list = []  # New list for std error
error_list = []  # New list for error
max_min_error_dict = {}

def ellipse_points(covariance, mean, num_points=100):
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    angle = np.linspace(0, 2*np.pi, num_points)
    ellipse = np.array([np.cos(angle), np.sin(angle)])
    transformation_matrix = np.sqrt(eigenvalues) * eigenvectors
    transformed_ellipse = np.dot(transformation_matrix, ellipse)
    return transformed_ellipse + mean.reshape((-1, 1))

def create_marker(points, frame_id, ns, marker_id, rgba, marker_type=Marker.LINE_STRIP):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.02
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    marker.points = points
    return marker

def calculate_distance(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def save_plots():
    global std_error_list, frame_list, error_list
    
    fig, ax = plt.subplots()
    
    ax.plot(frame_list, error_list, label='Error', color='g')
    ax.plot(frame_list, std_error_list, label='Uncertainty (Standard Deviation Error)', color='r')  # New line for std error
    
    ax.set_xlabel('Messages Received')
    ax.set_ylabel('Error (m)')
    ax.legend()
    ax.set_title('Error Fluctuation Along the Path and its Uncertainty')
    
    plt.savefig('error_and_std_error.png')
    plt.close(fig)

def callback(msg):
    global total_squared_error, total_error, num_of_errors, rmse_list, frame_list, max_min_error_dict

    try:
        transform_to_mocap = tf_buffer.lookup_transform('mocap', msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        transform_to_mocap_laser_link = tf_buffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return

    # Ellipse (circle) visualization
    position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    cov_matrix = np.array([[msg.pose.covariance[0], msg.pose.covariance[1]],
                           [msg.pose.covariance[6], msg.pose.covariance[7]]])
    
    points_np = ellipse_points(cov_matrix, position)

    # Transforming ellipse points to mocap frame
    transformed_points_msg = []
    for point in points_np.T:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.orientation.w = 1.0  
        transformed_point = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_to_mocap).pose.position
        transformed_points_msg.append(Point(transformed_point.x, transformed_point.y, transformed_point.z))


    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose
    
    pose_transformed_mocap = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_to_mocap)

    # Include the estimated position as the last point to close the ellipse
    transformed_points_msg.append(transformed_points_msg[0])  
    
    marker = create_marker(transformed_points_msg, 'mocap', 'covariance_ellipse', 0, [0, 1, 0, 0.5])
    marker_pub.publish(marker)

    # Print the real position
    real_position = Point()
    real_position.x = transform_to_mocap_laser_link.transform.translation.x
    real_position.y = transform_to_mocap_laser_link.transform.translation.y
    
    #rospy.loginfo("Real Position in mocap frame:")
    #rospy.loginfo("X: {:.4f}, Y: {:.4f}".format(real_position.x, real_position.y))
    
    # Calculate distances from real position to all ellipse points, and identify closest and furthest
    min_distance = float('inf')
    max_distance = float('-inf')
    closest_point = None
    furthest_point = None

    for point in transformed_points_msg[:-1]:
        distance = calculate_distance(real_position, point)
        if distance < min_distance:
            min_distance = distance
            closest_point = point
        if distance > max_distance:
            max_distance = distance
            furthest_point = point
    
    #rospy.loginfo("Min error: {:.4f}".format(min_distance))
    #rospy.loginfo("Max error: {:.4f}".format(max_distance))

    marker_pub.publish(create_marker([closest_point], 'mocap', 'closest_point', 1, [1, 0, 0, 1], Marker.SPHERE))  
    marker_pub.publish(create_marker([furthest_point], 'mocap', 'furthest_point', 2, [0, 0, 1, 1], Marker.SPHERE)) 
    
    if 85 <= msg.header.seq <= 1225:
        error_x = pose_transformed_mocap.pose.position.x - transform_to_mocap_laser_link.transform.translation.x
        error_y = pose_transformed_mocap.pose.position.y - transform_to_mocap_laser_link.transform.translation.y
        
        error = error_x**2 + error_y**2
        
        error_list.append(math.sqrt(error))  # Adding error to the list
        
        total_squared_error += error
        num_of_errors += 1
        total_error += math.sqrt(error)

        # Compute standard deviation of error from covariance matrix
        cov_matrix = np.array([[msg.pose.covariance[0], msg.pose.covariance[1]],
                               [msg.pose.covariance[6], msg.pose.covariance[7]]])
        std_error = np.sqrt(np.trace(cov_matrix) / 2)  # Trace of cov matrix divided by 2 (number of dimensions), then square root
        std_error_list.append(std_error)  # Adding std error to the list

        rmse = math.sqrt(total_squared_error / num_of_errors)
        mean = total_error / num_of_errors
        rospy.loginfo("Root Mean Square Error (RMSE) from frame 85 to [{}]: {:.4f}".format(msg.header.seq, rmse))
        rospy.loginfo("Mean Error: {:.4f}".format(mean))
        rospy.loginfo("Error: {:.4f}".format(math.sqrt(error)))

        rmse_list.append(rmse)
        frame_list.append(msg.header.seq)

        if msg.header.seq in [100, 300, 500, 700, 900, 1100]:
            max_min_error_dict[msg.header.seq] = (min_distance, max_distance)
            rospy.loginfo("Frame {}: Min Error: {:.4f}, Max Error: {:.4f}".format(msg.header.seq, min_distance, max_distance))
        
        if msg.header.seq == 1225:
            save_plots()
    
    odom_transformed = Odometry()
    odom_transformed.header.stamp = rospy.Time.now()
    odom_transformed.header.frame_id = 'mocap'
    odom_transformed.pose.pose = pose_transformed_mocap.pose
    
    pub.publish(odom_transformed)

if __name__ == '__main__':
    rospy.init_node('odometry_transformer')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/odometry/filtered', Odometry, callback)
    
    pub = rospy.Publisher('/odometry/filtered_mocap', Odometry, queue_size=10)
    marker_pub = rospy.Publisher('/covariance_ellipse', Marker, queue_size=10)
    
    rospy.spin()
