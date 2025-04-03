#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal  # Correct message type for /move_base/goal
import os

# File paths
goal_file = os.path.expanduser("/home/humanoidrobots/pepper/src/navigation/scripts/goal_log.txt")
followed_goal_file = os.path.expanduser("/home/humanoidrobots/pepper/src/navigation/scripts/followed_goals_log.txt")

def save_pose_to_file(msg, file_path, data_type):
    """Save pose data to a file with timestamp."""
    timestamp = rospy.get_time()
    x, y = msg.pose.position.x, msg.pose.position.y
    log_entry = "\n[{}] {}:\n{}, {}\n".format(timestamp, data_type, x, y)

    with open(file_path, "a") as file:
        file.write(log_entry)

    rospy.loginfo("{} logged to {}".format(data_type, file_path))

def goal_callback(msg):
    """Logs the final goal position from MoveBaseActionGoal."""
    pose = msg.goal.target_pose  # Extract the PoseStamped part
    save_pose_to_file(pose, goal_file, "Final Goal")

def followed_goal_callback(msg):
    """Logs followed/intermediate goals."""
    save_pose_to_file(msg, followed_goal_file, "Followed Goal")

def pose_logger():
    """Initialize ROS node and subscribe to relevant topics."""
    rospy.init_node("pose_logger", anonymous=True)

    # Subscribe to goal topics
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goal_callback)  # Fix message type
    rospy.Subscriber("/move_base/current_goal", PoseStamped, followed_goal_callback)

    rospy.loginfo("Pose Logger Node Started. Listening for goal updates...")
    rospy.spin()

if __name__ == "__main__":
    pose_logger()


