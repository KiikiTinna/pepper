#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Header

class ObstacleExtractor:
    def __init__(self):
        rospy.init_node("obstacle_extractor", anonymous=True)

        # Directly set the threshold for obstacles (occupied cells)
        self.occupied_thresh = 65  # 65% occupied (values > 65 are considered obstacles)
        self.map_topic = "/map"  # Topic for the map data
        self.obstacle_topic = "/obstacle_positions"  # Topic to publish obstacle positions
        
        # Create a publisher for obstacle positions
        self.obstacle_pub = rospy.Publisher(self.obstacle_topic, Point, queue_size=10)

        # Subscribe to the map topic
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)

        rospy.loginfo("ObstacleExtractor initialized.")

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        # Loop through all cells in the map
        for i in range(width * height):
            if msg.data[i] >= self.occupied_thresh:  # If the cell is occupied (obstacle)
                # Calculate the position (x, y) of the obstacle
                x = (i % width) * resolution + origin.position.x
                y = (i // width) * resolution + origin.position.y
                z = 0  # Assuming obstacles are at the ground level

                # Create a Point message to publish the obstacle position
                obstacle_point = Point(x, y, z)
                
                # Publish the obstacle position
                self.obstacle_pub.publish(obstacle_point)

                rospy.loginfo("Obstacle at position: (%f, %f, %f)" % (x, y, z))

if __name__ == "__main__":
    try:
        extractor = ObstacleExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

