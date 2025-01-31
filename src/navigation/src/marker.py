#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def publish_marker():
    rospy.init_node('simple_marker_publisher')
    marker_pub = rospy.Publisher('/simple_marker', Marker, queue_size=10)
    rospy.sleep(1)  # Wait for subscribers to connect

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "simple_shapes"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = 1.0
    marker.pose.position.y = 1.0
    marker.pose.position.z = 0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Fully opaque

    # Publish the marker
    marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        publish_marker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

