#include <right_side_layer/right_side_layer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(right_side_layer_namespace::RightSideLayer, costmap_2d::Layer)

namespace right_side_layer_namespace
{

RightSideLayer::RightSideLayer() : 
    right_side_bias_factor_(100.0),  // High cost bias
    obstacle_detection_range_(1.0),  // 1 meters detection range
    pose_received_(false)
{}

void RightSideLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = costmap_2d::NO_INFORMATION;
    
    // Load configurable parameters
    nh.param("right_side_bias_factor", right_side_bias_factor_, 100.0);
    nh.param("obstacle_detection_range", obstacle_detection_range_, 1.0);
    
    // Subscribe to robot pose
    ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 1, &RightSideLayer::poseCallback, this);
    
    matchSize();
}

void RightSideLayer::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
    pose_received_ = true;
}

void RightSideLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                   double* min_x, double* min_y, 
                                   double* max_x, double* max_y)
{
    *min_x = -std::numeric_limits<double>::max();
    *min_y = -std::numeric_limits<double>::max();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
}

void RightSideLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                                  int min_i, int min_j, 
                                  int max_i, int max_j)
{
    if (!pose_received_)
        return;

    // Get current robot pose and orientation
    double robot_x = current_pose_.pose.pose.position.x;
    double robot_y = current_pose_.pose.pose.position.y;
    double robot_yaw = tf::getYaw(current_pose_.pose.pose.orientation);

    unsigned int robot_cell_x, robot_cell_y;
    if (!master_grid.worldToMap(robot_x, robot_y, robot_cell_x, robot_cell_y))
        return;

    // Iterate through costmap cells
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            // Check for obstacles
            unsigned char cost = master_grid.getCost(i, j);
            if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                // Convert cell to world coordinates
                double obstacle_x, obstacle_y;
                master_grid.mapToWorld(i, j, obstacle_x, obstacle_y);

                // Calculate distance and angle to obstacle
                double dx = obstacle_x - robot_x;
                double dy = obstacle_y - robot_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                double angle_to_obstacle = std::atan2(dy, dx);

                // Only process obstacles within detection range
                if (distance <= obstacle_detection_range_) {
                    // Compute relative angle considering robot's orientation
                    double relative_angle = angles::shortest_angular_distance(robot_yaw, angle_to_obstacle);

                    // Focus on obstacles directly in front or near front of robot
                    if (std::abs(relative_angle) < M_PI/2) {
                        if (relative_angle < 0) {         
                            unsigned char current_cost = master_grid.getCost(i, j);
                            unsigned int new_cost = std::min(
                                static_cast<unsigned int>(current_cost) + static_cast<unsigned int>(right_side_bias_factor_),
                                static_cast<unsigned int>(costmap_2d::LETHAL_OBSTACLE - 1)
                            );
                            master_grid.setCost(i, j, new_cost);
                        }
                    }
                }
            }
        }
    }
}

void RightSideLayer::reset()
{
    pose_received_ = false;
}

void RightSideLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

} // namespace right_side_layer_namespace
