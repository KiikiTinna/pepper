#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


# Define the low area boundaries based on your specified points
AREA_MIN_X = -0.5
AREA_MAX_X = 0.5 
AREA_MIN_Y = -0.5 
AREA_MAX_Y = 2.0

SPEED_SCALING = 0.5
lower_speed_zone = False
pub_low = None


def pose_callback(pose_msg):
    global lower_speed_zone
    # Get current robot position from the pose message
    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y

    # Check if the robot is inside the defined rectangular area
    if (AREA_MIN_X <= x <= AREA_MAX_X) and (AREA_MIN_Y <= y <= AREA_MAX_Y):
        lower_speed_zone = True
    else:
        lower_speed_zone = False

def cmd_vel_callback(cmd_msg):
    if lower_speed_zone:
        low_speed_cmd = Twist()
        low_speed_cmd.linear.x  = cmd_msg.linear.x  * SPEED_SCALING
        low_speed_cmd.linear.y  = cmd_msg.linear.y  * SPEED_SCALING
        low_speed_cmd.linear.z  = cmd_msg.linear.z  * SPEED_SCALING
        low_speed_cmd.angular.x = cmd_msg.angular.x * SPEED_SCALING
        low_speed_cmd.angular.y = cmd_msg.angular.y * SPEED_SCALING
        low_speed_cmd.angular.z = cmd_msg.angular.z * SPEED_SCALING
        pub_low.publish(low_speed_cmd)
        rospy.loginfo("In speed zone: Scaling velocities by factor %.2f", SPEED_SCALING)

def main():
    global pub_low
    rospy.init_node('speed_zone_node')

    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

    rospy.Subscriber('/move_base_cmd_vel', Twist, cmd_vel_callback)

    pub_low = rospy.Publisher('/low_speed_cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()

