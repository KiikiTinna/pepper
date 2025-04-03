#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import os

# File paths
global_path_file = os.path.expanduser("/home/humanoidrobots/pepper/src/navigation/scripts/global_path_log.txt")
local_path_file = os.path.expanduser("/home/humanoidrobots/pepper/src/navigation/scripts/local_path_log.txt")

# Store last paths for comparison
previous_global_path = None
previous_local_path = None

def save_path_to_file(path_msg, file_path, path_type, previous_path):
    """Function to save path data to a file only if it changes, otherwise write 'SAME AS LAST'."""
    timestamp = rospy.get_time()

    if is_path_different(path_msg, previous_path):  # Log new path if different
        log_entry = "\n[{}] {} Path:\n".format(timestamp, path_type)
        for pose in path_msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            log_entry += "{}, {}\n".format(x, y)
    else:
        log_entry = "[{}] {} Path: SAME AS LAST\n".format(timestamp, path_type)

    with open(file_path, "a") as file:
        file.write(log_entry)

    rospy.loginfo("{} Path logged to {}".format(path_type, file_path))
    return path_msg  # Update previous path

def is_path_different(new_path, previous_path):
    """Compare current path with the previous one to check for changes."""
    if previous_path is None:
        return True  # Always log first path

    if len(new_path.poses) != len(previous_path.poses):
        return True  # Different number of points

    for i in range(len(new_path.poses)):
        if new_path.poses[i].pose.position.x != previous_path.poses[i].pose.position.x or \
           new_path.poses[i].pose.position.y != previous_path.poses[i].pose.position.y:
            return True  # A change in at least one point

    return False  # No significant change detected

def global_path_callback(msg):
    """Callback for Global Path updates."""
    global previous_global_path
    previous_global_path = save_path_to_file(msg, global_path_file, "Global", previous_global_path)

def local_path_callback(msg):
    """Callback for Local Path updates."""
    global previous_local_path
    previous_local_path = save_path_to_file(msg, local_path_file, "Local", previous_local_path)

def path_logger():
    """Initialize ROS node and subscribe to global and local path topics."""
    rospy.init_node("path_logger", anonymous=True)

    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, global_path_callback)
    rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, local_path_callback)

    rospy.loginfo("Path Logger Node Started. Listening for path updates...")
    rospy.spin()

if __name__ == "__main__":
    path_logger()

