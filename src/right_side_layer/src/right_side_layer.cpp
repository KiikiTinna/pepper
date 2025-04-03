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

PLUGINLIB_EXPORT_CLASS(right_side_layer::RightSideLayer, costmap_2d::Layer)

namespace right_side_layer
{

RightSideLayer::RightSideLayer() : initial_pose_received_(false), goal_received_(false), bias_factor_(80.0), bias_applied_(false), goal_direction_(0.0) {}

void RightSideLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    nh.param("bias_factor", bias_factor_, 200.0);
    nh.param("inflation_radius", inflation_radius_, 0.25);
    nh.param("outer_radius", outer_radius_, 0.5);  

    pose_sub_ = nh.subscribe("/amcl_pose", 1, &RightSideLayer::poseCallback, this);
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &RightSideLayer::goalCallback, this);
}

void RightSideLayer::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (!initial_pose_received_)
    {
        initial_pose_ = *msg;
        initial_pose_received_ = true;
        ROS_INFO("RightSideLayer: Initial pose received at (%.2f, %.2f)", 
                 initial_pose_.pose.pose.position.x, initial_pose_.pose.pose.position.y);
    }
}


void RightSideLayer::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose_ = *msg;
    goal_received_ = true;

    if (initial_pose_received_)
    {
        double dx = goal_pose_.pose.position.x - initial_pose_.pose.pose.position.x;
        double dy = goal_pose_.pose.position.y - initial_pose_.pose.pose.position.y;
        goal_direction_ = std::atan2(dy, dx);
        ROS_INFO("RightSideLayer: Goal received, direction angle set to %.2f radians", goal_direction_);
    }
}

void RightSideLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!initial_pose_received_ || !goal_received_ || bias_applied_)
        return;

    double start_x = initial_pose_.pose.pose.position.x;
    double start_y = initial_pose_.pose.pose.position.y;
    double goal_x = goal_pose_.pose.position.x;
    double goal_y = goal_pose_.pose.position.y;

    double padding = 0.3; //0.5 //0.3

    *min_x = std::min(start_x, goal_x);
    *min_y = std::min(start_y, goal_y) - padding;
    *max_x = std::max(start_x, goal_x);
    *max_y = std::max(start_y, goal_y) + padding;

    matchSize();

    ROS_INFO("RightSideLayer: updateBounds triggered once — bounds set from [%.2f, %.2f] to [%.2f, %.2f] ",*min_x, *min_y, *max_x, *max_y);
}


void RightSideLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, int max_i, int max_j)
{
    if (!initial_pose_received_ || !goal_received_ || bias_applied_)
        return;

    double robot_x = initial_pose_.pose.pose.position.x;
    double robot_y = initial_pose_.pose.pose.position.y;
    double goal_x = goal_pose_.pose.position.x;
    double goal_y = goal_pose_.pose.position.y;

    double path_angle = std::atan2(goal_y - robot_y, goal_x - robot_x);

    for (int j = min_j; j < max_j; ++j) {
        for (int i = min_i; i < max_i; ++i) {
            unsigned char cost = master_grid.getCost(i, j);

            if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;

            double obs_x, obs_y;
            master_grid.mapToWorld(i, j, obs_x, obs_y);

            for (double shift = 0.0; shift <= 1.0; shift += 0.05) {
                double bias_angle = path_angle + M_PI / 2.0;

                double right_x = obs_x + shift * std::cos(bias_angle);
                double right_y = obs_y + shift * std::sin(bias_angle);

                unsigned int mx, my;
                if (!master_grid.worldToMap(right_x, right_y, mx, my)) continue;

                unsigned char existing_cost = master_grid.getCost(mx, my);

                if (existing_cost == costmap_2d::NO_INFORMATION) continue;
                if (existing_cost >= costmap_2d::LETHAL_OBSTACLE) continue;

                int increased_cost = static_cast<int>(existing_cost) + static_cast<int>(bias_factor_);
                unsigned char new_cost = static_cast<unsigned char>(std::min(254, increased_cost));
                master_grid.setCost(mx, my, new_cost);
            }
        }
    }

    bias_applied_ = true; // ✅ Mark as done
    ROS_INFO("RightSideLayer: Bias applied once based on initial pose and goal.");
}


void RightSideLayer::resetLayer()
{
    
    initial_pose_received_ = false;
    goal_received_ = false;
    initial_pose_ = geometry_msgs::PoseWithCovarianceStamped();
    goal_pose_ = geometry_msgs::PoseStamped();

    matchSize();
    deactivate();
    activate();

    ROS_INFO("RightSideLayer: Layer reset complete.");
}




} // namespace right_side_layer
