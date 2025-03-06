#include "speed_filter_layer/speed_filter_layer.h"
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>
#include <costmap_2d/costmap_math.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <ros/subscriber.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(speed_filter_layer::SpeedFilterLayer, costmap_2d::Layer)

namespace speed_filter_layer
{

void SpeedFilterLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);

    std::string speed_map_file, yaml_file;
    nh.param("speed_map_image", speed_map_file, std::string(""));
    nh.param("speed_map_yaml", yaml_file, std::string(""));

    if(speed_map_file.empty() || yaml_file.empty()) {
        ROS_ERROR("You must specify 'speed_map_image' and 'speed_map_yaml'.");
        return;
    }

    // Load YAML metadata
    YAML::Node map_metadata = YAML::LoadFile(yaml_file);
    resolution_ = map_metadata["resolution"].as<double>();
    origin_x_ = map_metadata["origin"][0].as<double>();
    origin_y_ = map_metadata["origin"][1].as<double>();

    speed_map_ = cv::imread(speed_map_file, cv::IMREAD_GRAYSCALE);
    if (speed_map_.empty()) {
        ROS_ERROR("Cannot load speed map image at %s", speed_map_file.c_str());
        return;
    }
    map_width_ = speed_map_.cols;
    map_height_ = speed_map_.rows;
    
    ROS_INFO("Speed map loaded successfully.");

    // Setup ROS publishers/subscribers
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_filtered", 10);
    cmd_vel_sub_ = nh.subscribe("/cmd_vel", 10, &SpeedFilterLayer::cmdVelCallback, this);
}

// Function to get speed factor from speed map
double SpeedFilterLayer::getSpeedFactor(double x, double y)
{
    unsigned int mx, my;
    if (!layered_costmap_->getCostmap()->worldToMap(x, y, mx, my))
        return 1.0;  // Default to full speed if outside the map

    // Image has origin at top-left; costmap at bottom-left
    unsigned int img_y = map_height_ - my - 1;

    int pixel_value = speed_map_.at<uchar>(img_y, mx);

    if (pixel_value <= 50)
        return 0.3;   // Slowest speed
    else if (pixel_value <= 120)
        return 0.6;   // Medium speed
    else
        return 1.0;   // Full speed
}

// Callback for velocity commands
void SpeedFilterLayer::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_cmd_vel_ = *msg;
}

// Adjust speed based on the robot's current position
void SpeedFilterLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
{
    double speed_factor = getSpeedFactor(robot_x, robot_y);
    ROS_INFO_THROTTLE(2.0, "Current Speed Factor: %.2f", speed_factor);

    // Modify velocity command based on speed factor
    geometry_msgs::Twist filtered_cmd_vel = last_cmd_vel_;
    filtered_cmd_vel.linear.x *= speed_factor;

    // Publish the modified velocity
    vel_pub_.publish(filtered_cmd_vel);
}

// Adjust costmap (but allow traversal)
void SpeedFilterLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) return;

    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double resolution = costmap->getResolution();

    for (int i = min_i; i < max_i; i++) {
        for (int j = min_j; j < max_j; j++) {
            double wx, wy;
            costmap->mapToWorld(i, j, wx, wy);

            double speed_factor = getSpeedFactor(wx, wy);
            unsigned char cost = (1.0 - speed_factor) * 100;  // Increase cost in slow areas

            unsigned char old_cost = costmap->getCost(i, j);
            if (old_cost == costmap_2d::NO_INFORMATION) continue;

            costmap->setCost(i, j, std::max(cost, old_cost));
        }
    }
}

} // namespace speed_filter_layer

