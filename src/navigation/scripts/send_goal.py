#!/usr/bin/env python

#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from actionlib import SimpleActionClient

# Distance to move forward
GOAL_DISTANCE = 1.0  # Change this to adjust distance

def send_goal():
    """Sends a goal straight ahead from the robot's current position."""
    rospy.init_node("send_straight_goal", anonymous=True)

    # Wait for AMCL pose
    rospy.loginfo("Waiting for AMCL pose...")
    amcl_pose_msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)

    # Extract position
    x = amcl_pose_msg.pose.pose.position.x
    y = amcl_pose_msg.pose.pose.position.y

    # Extract orientation (quaternion)
    quaternion = (
        amcl_pose_msg.pose.pose.orientation.x,
        amcl_pose_msg.pose.pose.orientation.y,
        amcl_pose_msg.pose.pose.orientation.z,
        amcl_pose_msg.pose.pose.orientation.w
    )

    # Convert quaternion to yaw (rotation in 2D)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

    # Compute new goal position (moving straight ahead)
    goal_x = x + GOAL_DISTANCE * math.cos(yaw)
    goal_y = y + GOAL_DISTANCE * math.sin(yaw)

    # Create goal message
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    goal_msg.target_pose.header.frame_id = "map"
    goal_msg.target_pose.pose.position.x = goal_x
    goal_msg.target_pose.pose.position.y = goal_y
    goal_msg.target_pose.pose.orientation = amcl_pose_msg.pose.pose.orientation  # Keep same orientation

    # Create an action client to send the goal
    client = SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # Send the goal
    rospy.loginfo("Sending goal: x={}, y={}".format(goal_x, goal_y))
    client.send_goal(goal_msg)
    client.wait_for_result()

    rospy.loginfo("Goal execution complete!")

if __name__ == "__main__":
    send_goal()

