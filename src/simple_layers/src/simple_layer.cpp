#include <simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(simple_layers::SimpleLayer, costmap_2d::Layer)

namespace simple_layers {

SimpleLayer::SimpleLayer() : enabled_(true) { }

void SimpleLayer::onInitialize() {

    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    matchSize();
    
    // Initialize publisher for debugging
    ros::NodeHandle public_nh;
    custom_cost_pub_ = public_nh.advertise<nav_msgs::OccupancyGrid>("simple_layer/custom_costs", 1);
    
    std::string costly_zones_file, yaml_file;
    nh.param("costly_zones_image", costly_zones_file, std::string(""));
    nh.param("costly_zones_yaml", yaml_file, std::string(""));
    
    // Optional parameter for how high to set the cost (default: 200)
    int avoidance_cost;
    nh.param("avoidance_cost", avoidance_cost, 200);
    avoidance_cost_ = static_cast<unsigned char>(avoidance_cost);
    
    if (costly_zones_file.empty() || yaml_file.empty()) {
        ROS_WARN("No custom YAML or image path provided. Using default costmap.");
        return;
    }
    
    ROS_INFO("Loading custom cost map from: %s", costly_zones_file.c_str());
    ROS_INFO("Using map metadata from: %s", yaml_file.c_str());
    
    YAML::Node map_metadata = YAML::LoadFile(yaml_file);
    resolution_ = map_metadata["resolution"].as<double>();
    origin_x_ = map_metadata["origin"][0].as<double>();
    origin_y_ = map_metadata["origin"][1].as<double>();
    occupied_thresh_ = map_metadata["occupied_thresh"].as<double>();
    free_thresh_ = map_metadata["free_thresh"].as<double>();
    
    if (costly_zones_file[0] != '/') {
        std::string yaml_dir = yaml_file.substr(0, yaml_file.find_last_of("/"));
        costly_zones_file = yaml_dir + "/" + costly_zones_file;
        ROS_INFO("Adjusted image path to: %s", costly_zones_file.c_str());
    }
    
    costly_map_ = cv::imread(costly_zones_file, cv::IMREAD_GRAYSCALE);
    if (costly_map_.empty()) {
        ROS_ERROR("Cannot load cost map image at %s", costly_zones_file.c_str());
        return;
    }
    
    map_width_ = costly_map_.cols;
    map_height_ = costly_map_.rows;
    
    ROS_INFO("Loaded costly_map with dimensions: %dx%d", map_width_, map_height_);
    
    // Initialize custom_map_ with FREE_SPACE
    custom_map_ = std::vector<std::vector<unsigned char>>(map_height_, 
                 std::vector<unsigned char>(map_width_, costmap_2d::FREE_SPACE));
    
    // Analyze the values in the map
    std::map<int, int> value_counts;
    for (int y = 0; y < map_height_; y++) {
        for (int x = 0; x < map_width_; x++) {
            int val = (int)costly_map_.at<unsigned char>(y, x);
            value_counts[val]++;
        }
    }
    
    ROS_INFO("Values found in costly_map:");
    for (const auto& pair : value_counts) {
        ROS_INFO("Value %d: %d occurrences", pair.first, pair.second);
    }
    
    // Convert costly_map_ values to costmap values
    int count_costly = 0;
    for (int y = 0; y < map_height_; y++) {
        for (int x = 0; x < map_width_; x++) {
            unsigned char pixel_value = costly_map_.at<unsigned char>(y, x);
            
            // Look for any non-free, non-lethal value
            // Adjust this condition based on what values you found in your map
            if (pixel_value > 0 && pixel_value < 100) {
                custom_map_[y][x] = avoidance_cost_;
                count_costly++;
            }
        }
    }
    
    ROS_INFO("Custom cost map successfully loaded! Size: %dx%d", map_width_, map_height_);
    ROS_INFO("Found %d cells with costs in costly_map", count_costly);
    ROS_INFO("Areas with costs will have avoidance cost of %d", avoidance_cost_);
    
    // Publish the custom map immediately after initialization
    publishCustomCosts();
    
    // Create and publish a test grid to verify publisher works
    publishTestGrid();
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                              double* min_x, double* min_y, 
                              double* max_x, double* max_y) {
    if (!enabled_ || custom_map_.empty()) {
        ROS_DEBUG_THROTTLE(5.0, "SimpleLayer::updateBounds skipped (enabled=%d, custom_map_empty=%d)",
                          enabled_, custom_map_.empty());
        return;
    }
    
    // Update bounds to include the entire custom map
    *min_x = std::min(*min_x, origin_x_);
    *min_y = std::min(*min_y, origin_y_);
    *max_x = std::max(*max_x, origin_x_ + map_width_ * resolution_);
    *max_y = std::max(*max_y, origin_y_ + map_height_ * resolution_);
    
    ROS_DEBUG("SimpleLayer::updateBounds: bounds set to %.2f, %.2f, %.2f, %.2f", 
              *min_x, *min_y, *max_x, *max_y);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                             int min_i, int min_j, int max_i, int max_j) {
    ROS_INFO_THROTTLE(1.0, "SimpleLayer enabled: %s", enabled_ ? "true" : "false");
    
    if (!enabled_ || custom_map_.empty()) {
        ROS_DEBUG_THROTTLE(5.0, "SimpleLayer::updateCosts skipped (enabled=%d, custom_map_empty=%d)",
                          enabled_, custom_map_.empty());
        return;
    }
    
    // Log that we're updating costs
    ROS_INFO_THROTTLE(1.0, "SimpleLayer::updateCosts called with bounds: min_i=%d, min_j=%d, max_i=%d, max_j=%d", 
                     min_i, min_j, max_i, max_j);
    ROS_INFO_THROTTLE(1.0, "custom_map_ size: %dx%d", 
                     (int)custom_map_.size(), custom_map_.empty() ? 0 : (int)custom_map_[0].size());
    
    int count_updates = 0;
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            // Convert master grid cell to world coordinates
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);
            
            // Convert world coordinates to custom map coordinates
            int mx = static_cast<int>((wx - origin_x_) / resolution_);
            int my = static_cast<int>((wy - origin_y_) / resolution_);
            
            // Check if the point is within the custom map bounds
            if (mx >= 0 && mx < map_width_ && my >= 0 && my < map_height_) {
                unsigned char custom_cost = custom_map_[my][mx];
                
                // If the custom cost is higher than the current cost, update it
                if (custom_cost > costmap_2d::FREE_SPACE) {
                    unsigned char current = master_grid.getCost(i, j);
                    if (current < custom_cost) {
                        master_grid.setCost(i, j, custom_cost);
                        count_updates++;
                        
                        // Log some of the updates for debugging
                        if (count_updates % 100 == 0) {
                            ROS_INFO_THROTTLE(2.0, "Updated cost at world: (%.2f, %.2f), map: (%d, %d), from %d to %d", 
                                            wx, wy, mx, my, current, custom_cost);
                        }
                    }
                }
            }
        }
    }
    
    ROS_INFO_THROTTLE(1.0, "SimpleLayer::updateCosts updated %d cells", count_updates);
    
    // Publish the custom costs for visualization
    static ros::Time last_pub_time = ros::Time::now();
    ros::Duration time_since_last_pub = ros::Time::now() - last_pub_time;
    if (time_since_last_pub.toSec() > 2.0) {  // Publish every 2 seconds
        publishCustomCosts();
        last_pub_time = ros::Time::now();
    }
}

void SimpleLayer::publishCustomCosts() {
    ROS_INFO_THROTTLE(2.0, "Attempting to publish custom costs...");
    
    if (custom_map_.empty()) {
        ROS_WARN("Cannot publish custom costs: custom_map_ is empty");
        return;
    }
    
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";  // Make sure this matches your map frame
    grid.info.resolution = resolution_;
    grid.info.width = map_width_;
    grid.info.height = map_height_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    
    grid.data.resize(map_width_ * map_height_);
    for (unsigned int y = 0; y < map_height_; y++) {
        for (unsigned int x = 0; x < map_width_; x++) {
            unsigned int index = y * map_width_ + x;
            // Convert costmap values to occupancy grid values (0-100)
            if (custom_map_[y][x] > costmap_2d::FREE_SPACE) {
                grid.data[index] = 99;  // High cost but not obstacle
            } else {
                grid.data[index] = 0;   // Free space
            }
        }
    }
    
    custom_cost_pub_.publish(grid);
    ROS_INFO_THROTTLE(2.0, "Published custom costs to simple_layer/custom_costs");
}

void SimpleLayer::publishTestGrid() {
    // Create and publish a test grid
    nav_msgs::OccupancyGrid test_grid;
    test_grid.header.stamp = ros::Time::now();
    test_grid.header.frame_id = "map";
    test_grid.info.resolution = 0.05;
    test_grid.info.width = 100;
    test_grid.info.height = 100;
    test_grid.info.origin.position.x = 0;
    test_grid.info.origin.position.y = 0;
    
    test_grid.data.resize(100 * 100, 0);
    // Create a simple pattern
    for (int y = 25; y < 75; y++) {
        for (int x = 25; x < 75; x++) {
            test_grid.data[y * 100 + x] = 99;
        }
    }
    
    custom_cost_pub_.publish(test_grid);
    ROS_INFO("Published test grid to simple_layer/custom_costs");
}

} // namespace simple_layers
