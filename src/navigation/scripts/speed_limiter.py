#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SpeedLimiter:
    def __init__(self):
        rospy.init_node('speed_limiter')

        self.speed_limit = 1.0  # Default max speed

        rospy.Subscriber("/speed_limit", Float32, self.speed_limit_callback)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.cmd_vel_callback)

        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def speed_limit_callback(self, msg):
        self.speed_limit = msg.data
        rospy.loginfo("Updated speed limit: %.2f", self.speed_limit)

    def cmd_vel_callback(self, msg):
        # Apply speed limit
        msg.linear.x = min(msg.linear.x, self.speed_limit)
        self.pub_cmd_vel.publish(msg)

if __name__ == '__main__':
    SpeedLimiter()
    rospy.spin()
