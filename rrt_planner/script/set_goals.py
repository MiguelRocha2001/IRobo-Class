#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
import actionlib
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def send_goal(x, y, w, color_r, color_g, color_b):
    # Check for valid goal position
    if abs(x) > 100 or abs(y) > 100:
        rospy.logerr("Goal position is out of bounds")
        return False

    # Create a new goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, 0.0, w))

    # Publish the marker for visualization in RViz
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color = ColorRGBA(color_r, color_g, color_b, 1.0)
    marker.pose = goal.target_pose.pose
    marker_pub.publish(marker)

    # Send goal and wait for result
    move_base.send_goal(goal)
    finished = move_base.wait_for_result(rospy.Duration(60))

    if not finished:
        move_base.cancel_goal()
        rospy.logerr("Timed out achieving goal")
        return False

    state = move_base.get_state()

    if state == GoalStatus.SUCCEEDED:
        return True
    else:
        rospy.logerr("Failed to reach the goal")
        return False

if __name__ == '__main__':
    rospy.init_node('goal_publisher')

    # Initialize marker publisher
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Initialize move_base client
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(60))

    goals = [
        {"x": 0.0, "y": -0.5, "w": 1.0, "r": 1.0, "g": 0.0, "b": 0.0},
        {"x": 1.0, "y": 0.5, "w": 1.0, "r": 0.0, "g": 1.0, "b": 0.0},
        {"x": -0.5, "y": -1.5, "w": 1.0, "r": 0.0, "g": 0.0, "b": 1.0},
    ]

    for goal in goals:
        success = send_goal(goal["x"], goal["y"], goal["w"], goal["r"], goal["g"], goal["b"])
        if not success:
            rospy.logerr("Stopping goal sequence")
            break
