#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class LaserInterpolator:
    def __init__(self):
        rospy.init_node("laser_republisher", anonymous=True)
        self.pub = rospy.Publisher("/laser_scan_fresh", LaserScan, queue_size=10)
        rospy.Subscriber("/laser/srd_front/scan", LaserScan, self.scan_callback)
        self.latest_scan = None
        self.rate = rospy.Rate(5)  # Fake a 5 Hz update rate

    def scan_callback(self, msg):
        self.latest_scan = msg  # Store the latest scan data

    def publish_interpolated_scan(self):
        while not rospy.is_shutdown():
            if self.latest_scan:
                new_scan = self.latest_scan
                new_scan.header.stamp = rospy.Time.now()  # Update timestamp
                self.pub.publish(new_scan)  # Publish as if it's new
            self.rate.sleep()  # Maintain 5 Hz update rate

if __name__ == "__main__":
    node = LaserInterpolator()
    node.publish_interpolated_scan()

