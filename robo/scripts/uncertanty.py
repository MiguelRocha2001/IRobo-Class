import rospy
from nav_msgs.msg import Odometry
import numpy as np

def callback(data):
    # Access the covariance matrix
    covariance_matrix = data.pose.covariance

    # Define the number of dimensions (e.g., 6 for x, y, z, roll, pitch, yaw)
    num_dimensions = 6

    # Calculate and print the uncertainties for each dimension
    for i in range(num_dimensions):
        # Extract the relevant variance element for the dimension
        var_i = covariance_matrix[i * (num_dimensions + 1)]  # Diagonal element for dimension i

        # Calculate the standard deviation (uncertainty) for the dimension
        uncertainty_i = np.sqrt(var_i)

        # Print the uncertainty for the dimension
        variable = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        print('Uncertainty for ' + variable[i] + ': ' + str(uncertainty_i))

rospy.init_node('covariance_subscriber')
rospy.Subscriber('/odometry/filtered', Odometry, callback)
rospy.spin()
