#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <limits>
#include <cmath>
#include <ros/ros.h>
#include <simple_layers/simple_layer.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() : min_obstacle_distance_(5.0), goal_received_(false), goal_reached_(false), enabled_(true) {}

void SimpleLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    nh.param("enabled", enabled_, true);
    nh.param("min_speed", min_speed_, 0.4);
    nh.param("max_speed", max_speed_, 0.9);
    nh.param("obstacle_distance_threshold", obstacle_distance_threshold_, 1.0);

    laser_sub_ = nh.subscribe("/laser/srd_front/scan", 10, &SimpleLayer::laserCallback, this);
    speed_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    goal_status_sub_ = nh.subscribe("/move_base/status", 10, &SimpleLayer::goalStatusCallback, this);
    global_plan_sub_ = nh.subscribe("/move_base/NavfnROS/plan", 10, &SimpleLayer::globalPlanCallback, this);

    update_timer_ = nh.createTimer(ros::Duration(0.2), &SimpleLayer::updateMap, this);
}

void SimpleLayer::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (msg->ranges.empty())
    {
        ROS_WARN_THROTTLE(1.0, "SimpleLayer: Received empty laser scan!");
        return;
    }

    min_obstacle_distance_ = msg->range_max;
    for (const float &range : msg->ranges)
    {
        if (range >= msg->range_min && range <= msg->range_max)
        {
            min_obstacle_distance_ = std::min(min_obstacle_distance_, static_cast<double>(range));
        }
    }

    adjustSpeed();
}

void SimpleLayer::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    if (!msg->status_list.empty())
    {
        int status = msg->status_list.back().status;
        if (status == 3)  // Goal reached
        {
            goal_reached_ = true;
            goal_received_ = false;
            ROS_INFO("SimpleLayer: Goal reached! Stopping robot.");
        }
    }
}

void SimpleLayer::globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    global_plan_ = msg->poses;  
    goal_received_ = !global_plan_.empty();  // ✅ Robot moves only when path is available
    ROS_INFO("SimpleLayer: Received new global plan with %lu points", global_plan_.size());
    adjustSpeed();
}

void SimpleLayer::adjustSpeed()
{
    geometry_msgs::Twist vel_msg;

    if (!goal_received_)  
    {
        vel_msg.linear.x = 0.0;  // 🚨 No path → Stop robot
    }
    else if (goal_reached_)  
    {
        vel_msg.linear.x = 0.0;  // 🚨 Goal reached → Stop robot
        goal_received_ = false;
        goal_reached_ = false;
    }
    else  
    {
        double min_distance_to_path = std::numeric_limits<double>::max();
        
        for (const auto& pose : global_plan_)
        {
            double dx = pose.pose.position.x - 0.0;
            double dy = pose.pose.position.y - 0.0;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance_to_path)
            {
                min_distance_to_path = distance;
            }
        }

        if (min_obstacle_distance_ < obstacle_distance_threshold_)
        {
            vel_msg.linear.x = min_speed_;
        }
        else if (min_distance_to_path > 1.0)  
        {
            vel_msg.linear.x = min_speed_;
        }
        else  
        {
            vel_msg.linear.x = max_speed_;
        }
    }

    speed_pub_.publish(vel_msg);
    ROS_INFO_THROTTLE(1.0, "SimpleLayer: Speed updated to: %.2f", vel_msg.linear.x);
}

void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                               double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
        return;

    ROS_INFO_THROTTLE(1.0, "SimpleLayer: Expanding costmap bounds.");

    *min_x = std::min(*min_x, origin_x - 1.0);
    *min_y = std::min(*min_y, origin_y - 1.0);
    *max_x = std::max(*max_x, origin_x + 1.0);
    *max_y = std::max(*max_y, origin_y + 1.0);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    for (int i = min_i; i < max_i; i++)
    {
        for (int j = min_j; j < max_j; j++)
        {
            master_grid.setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
        }
    }

    ROS_INFO_THROTTLE(1.0, "SimpleLayer: Updated costs in local costmap.");
}

void SimpleLayer::updateMap(const ros::TimerEvent& event)
{
    if (!enabled_)
        return;

    double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
    
    this->matchSize();
    this->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
    this->updateCosts(*layered_costmap_->getCostmap(), 0, 0, 
                      layered_costmap_->getCostmap()->getSizeInCellsX(), 
                      layered_costmap_->getCostmap()->getSizeInCellsY());

    ROS_INFO_THROTTLE(1.0, "SimpleLayer: Costmap manually updated.");
}

} // namespace simple_layer_namespace


