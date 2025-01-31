#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker

class SpeedMapGenerator:
    def __init__(self):
        rospy.init_node("costmap_speed_map", anonymous=True)

        # Parameters
        self.obstacle_influence_radius = rospy.get_param("~obstacle_influence_radius", 1.5)
        self.min_speed = rospy.get_param("~min_speed", 0.2)
        self.max_speed = rospy.get_param("~max_speed", 0.8)

        # Subscribers and Publishers
        self.costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.costmap_callback)
        self.speed_map_pub = rospy.Publisher("/speed_map", OccupancyGrid, queue_size=1)
        self.marker_pub = rospy.Publisher("/speed_zones", MarkerArray, queue_size=1)

        rospy.loginfo("SpeedMapGenerator initialized.")

    def costmap_callback(self, costmap):
        speed_map = OccupancyGrid()
        speed_map.header = costmap.header
        speed_map.info = costmap.info

        # Calculate speed zones
        data = []
        for i, value in enumerate(costmap.data):
            if value == 100:  # Obstacle
                data.append(int(self.min_speed * 100))  # Convert to int for simplicity
            elif value >= 0:  # Free space
                data.append(int(self.max_speed * 100))
            else:  # Unknown
                data.append(-1)
        speed_map.data = data

        # Publish speed map
        self.speed_map_pub.publish(speed_map)

        # Visualize in RViz
        self.publish_speed_markers(costmap)

    def publish_speed_markers(self, costmap):
        marker_array = MarkerArray()
        for i, value in enumerate(costmap.data):
            if value == 100:  # Obstacle
                marker = Marker()
                marker.header = costmap.header
                marker.ns = "speed_zones"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.scale.x = marker.scale.y = self.obstacle_influence_radius * 2
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                marker.pose.position.x = (i % costmap.info.width) * costmap.info.resolution
                marker.pose.position.y = (i // costmap.info.width) * costmap.info.resolution
                marker.pose.position.z = 0
                marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    try:
        SpeedMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

