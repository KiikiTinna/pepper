#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Define the boundaries of the stop area (adjust these values to your area)
STOP_AREA_X_MIN = -1.1
STOP_AREA_X_MAX = -0.2
STOP_AREA_Y_MIN = -0.8
STOP_AREA_Y_MAX = 0.1

# Flag to ensure we only trigger the stop once per area entry
stopped = False

def pose_callback(pose_msg):
    global stopped
    # Extract robot's current position from the pose message
    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y

    # Check if the robot is inside the designated stop area
    if (STOP_AREA_X_MIN <= x <= STOP_AREA_X_MAX) and (STOP_AREA_Y_MIN <= y <= STOP_AREA_Y_MAX):
        if not stopped:
            rospy.loginfo("Robot has entered the stop area. Stopping for 2 seconds.")
            # Publish zero velocities to stop the robot
            stop_twist = Twist()
            pub.publish(stop_twist)
            # Set the flag so we don't repeatedly trigger while inside the area
            stopped = True
            # Sleep for 2 seconds while the robot is stopped
            rospy.sleep(2.0)
            rospy.loginfo("Resuming operation after stop.")
    else:
        # Reset the flag when the robot leaves the area
        stopped = False

def main():
    global pub
    rospy.init_node('stop_in_area_node')

    # Subscribe to the robot's pose topic (adjust topic name if needed)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

    # Publisher to send velocity commands (adjust topic if necessary)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()

