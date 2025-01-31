#!/usr/bin/env python


import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from math import sqrt

class SpeedMapGenerator:
    def __init__(self):
        rospy.init_node("speed_map_generator", anonymous=True)

        # Parameters
        self.obstacle_influence_radius = rospy.get_param("~obstacle_influence_radius", 1.5)
        self.min_speed = rospy.get_param("~min_speed", 0.2)
        self.max_speed = rospy.get_param("~max_speed", 0.8)

        # Topics
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/obstacle_positions")
        self.costmap_pub = rospy.Publisher("/speed_map", OccupancyGrid, queue_size=1)
        self.marker_pub = rospy.Publisher("/speed_zones", MarkerArray, queue_size=1)

        # Obstacle positions
        self.obstacles = []

        # Subscribe to obstacle positions
        rospy.Subscriber(self.obstacle_topic, Point, self.obstacle_callback)

        rospy.loginfo("SpeedMapGenerator initialized.")

    def obstacle_callback(self, msg):
        # Add obstacle to the list
        self.obstacles.append((msg.x, msg.y))

        # Generate and publish the speed map
        self.generate_speed_map()

    def generate_speed_map(self):
        if not self.obstacles:
            rospy.logwarn("No obstacles to generate the speed map.")
            return

        # Create an OccupancyGrid for the speed map
        speed_map = OccupancyGrid()
        speed_map.header.frame_id = "map"
        speed_map.header.stamp = rospy.Time.now()
        speed_map.info.resolution = 0.1  # 10 cm per grid cell
        speed_map.info.width = 100  # 10 meters wide
        speed_map.info.height = 100  # 10 meters tall
        speed_map.info.origin.position.x = -5.0  # Center map at origin
        speed_map.info.origin.position.y = -5.0

        # Initialize the grid with max speed values
        grid_size = speed_map.info.width * speed_map.info.height
        speed_map.data = [int(self.max_speed * 100)] * grid_size

        # Update the grid based on obstacle influence
        for obs_x, obs_y in self.obstacles:
            self.update_speed_map(speed_map, obs_x, obs_y)

        # Publish the speed map
        self.costmap_pub.publish(speed_map)

        # Publish markers for visualization
        self.publish_speed_markers()

    def update_speed_map(self, speed_map, obs_x, obs_y):
        """
        Update the speed map grid based on an obstacle's influence.
        """
        for y in range(speed_map.info.height):
            for x in range(speed_map.info.width):
                # Convert grid indices to world coordinates
                cell_x = speed_map.info.origin.position.x + x * speed_map.info.resolution
                cell_y = speed_map.info.origin.position.y + y * speed_map.info.resolution

                # Calculate distance from the obstacle
                distance = sqrt((cell_x - obs_x) ** 2 + (cell_y - obs_y) ** 2)

                # Adjust speed based on distance
                if distance <= self.obstacle_influence_radius:
                    index = y * speed_map.info.width + x
                    speed_factor = max(self.min_speed, self.max_speed * (distance / self.obstacle_influence_radius))
                    speed_map.data[index] = int(speed_factor * 100)

    def publish_speed_markers(self):
        """
        Publish markers to visualize speed zones around obstacles in RViz.
        """
        marker_array = MarkerArray()
        for idx, (obs_x, obs_y) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "speed_zones"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = self.obstacle_influence_radius * 2
            marker.scale.y = self.obstacle_influence_radius * 2
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.pose.position.x = obs_x
            marker.pose.position.y = obs_y
            marker.pose.position.z = 0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

if __name__ == "__main__":
    try:
        SpeedMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

