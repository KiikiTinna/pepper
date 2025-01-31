#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose():
    rospy.init_node('set_initial_pose', anonymous=True)
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # Wait for the publisher to connect
    rospy.sleep(1)

    # Define the pose
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "/map"
    initial_pose.header.stamp = rospy.Time.now()

    # Set position
    initial_pose.pose.pose.position.x = -1.334
    initial_pose.pose.pose.position.y = 0.561
    initial_pose.pose.pose.position.z = 0.0

    # Set orientation (as quaternion)
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.984
    initial_pose.pose.pose.orientation.w = 0.178

    # Set covariance
    initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

    # Publish the pose
    rospy.loginfo("Publishing initial pose...")
    initial_pose_pub.publish(initial_pose)
    rospy.sleep(1)  

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass


