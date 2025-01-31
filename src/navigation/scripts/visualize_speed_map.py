#!/usr/bin/env python


import rospy
from visualization_msgs.msg import MarkerArray

def callback(marker_array):
    rospy.loginfo("Received {} markers.".format(len(marker_array.markers)))

def main():
    rospy.init_node('visualize_speed_map')
    rospy.Subscriber('/marker_array_topic', MarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

