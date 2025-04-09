#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Define the stop area boundaries based on your specified points
AREA_MIN_X = -1.0
AREA_MAX_X = -0.79
AREA_MIN_Y = -0.25
AREA_MAX_Y = 1.25

# Global flag to ensure the stop is triggered only once per pass
stopped = False

def pose_callback(pose_msg):
    global stopped
    # Get current robot position from the pose message
    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y

    # Check if the robot is inside the defined rectangular area
    if (AREA_MIN_X <= x <= AREA_MAX_X) and (AREA_MIN_Y <= y <= AREA_MAX_Y):
        if not stopped:
            rospy.loginfo("Robot entered the stop area. Stopping for 5 seconds.")
            stop_twist = Twist()  # All velocities are zero by default
            pub_stop.publish(stop_twist)
            stopped = True
            rospy.sleep(5.0)  # Pause for 2 seconds
            rospy.loginfo("Stop period ended. Resuming operation.")
    else:
        # Reset the flag when the robot leaves the area
        stopped = False

def main():
    global pub_stop
    rospy.init_node('stop_area_node')
    
    # Subscribe to the robot's pose (usually published by AMCL)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    
    # Publisher for stop commands (ensure this topic is prioritized in your multiplexer)
    pub_stop = rospy.Publisher('/stop_cmd_vel', Twist, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()

