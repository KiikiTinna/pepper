#include "speed_filter_layer/speed_filter_layer.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <costmap_2d/costmap_math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <limits>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>


PLUGINLIB_EXPORT_CLASS(speed_filter_layer::SpeedFilterLayer, costmap_2d::Layer)

namespace speed_filter_layer
{
SpeedFilterLayer::SpeedFilterLayer() : speed_limit_(1.0), slow_down_zone_radius_(2.0), min_obstacle_distance_(3.0) {}

void SpeedFilterLayer::onInitialize()
{
    ros::NodeHandle private_nh("~/" + name_);
    private_nh.param("slow_down_zone_radius", slow_down_zone_radius_, 2.0);
    private_nh.param("speed_limit", speed_limit_, 0.7);

    speed_pub_ = nh_.advertise<std_msgs::Float32>("/speed_limit", 10);
    distance_pub_ = nh_.advertise<std_msgs::Float32>("/distance_to_obstacle", 10);  
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_filtered", 10, true);


    laser_sub_ = nh_.subscribe("/pepper_robot/naoqi_driver/laser", 10, &SpeedFilterLayer::laserCallback, this);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &SpeedFilterLayer::cmdVelCallback, this);
    local_plan_sub_ = nh_.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 10, &SpeedFilterLayer::localPlanCallback, this);




    is_local_plan_received_ = false;
    current_ = true;
}


void SpeedFilterLayer::updateMinVelocity()
{
    ros::NodeHandle nh;

    if (!ros::service::exists("/move_base/TrajectoryPlannerROS/set_parameters", true))
    {
        ROS_WARN_THROTTLE(5.0, "Dynamic reconfigure service for move_base is not available. Skipping min_vel_x update.");
        return;
    }

    double existing_min_vel_x;
    if (!nh.getParam("/move_base/TrajectoryPlannerROS/min_vel_x", existing_min_vel_x))
    {
        existing_min_vel_x = 0.1;  // Default fallback
    }

    if (std::fabs(speed_limit_ - existing_min_vel_x) < 0.01) 
    {
        return; // ✅ Skip update if the value hasn't changed significantly
    }

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "min_vel_x";
    double_param.value = speed_limit_;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;

    if (ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp))
    {
        ROS_INFO_THROTTLE(5.0, "Updated min_vel_x to: %.2f", speed_limit_);
    }
    else
    {
        ROS_WARN_THROTTLE(5.0, "Failed to update min_vel_x in move_base.");
    }
}




void SpeedFilterLayer::localPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (!msg->poses.empty())  
    {
        is_local_plan_received_ = true;
        last_local_plan_time_ = ros::Time::now();  // ✅ Store the timestamp
        ROS_INFO_THROTTLE(5.0, "Local plan received, activating SpeedFilterLayer.");
    }
}


bool SpeedFilterLayer::isLocalPlanValid()
{
    return (ros::Time::now() - last_local_plan_time_).toSec() < 2.0;  // ✅ Keep local plan for 2 seconds
}


void SpeedFilterLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                    double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!isLocalPlanValid())  
    {
        ROS_WARN_THROTTLE(2.0, "SpeedFilterLayer skipped - No valid local plan.");
        return;
    }
    // ✅ Continue normal update if plan is still valid
}



void SpeedFilterLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
     // Continue cost updates normally if a local plan exists
}



void SpeedFilterLayer::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!is_local_plan_received_)  //  Don't process if no local plan is available
    {
        ROS_WARN("Skipping laserCallback - No global plan received.");
        return;
    }

    int front_index = round((0.0 - msg->angle_min) / msg->angle_increment);
    int left_10_index = round((0.1745 - msg->angle_min) / msg->angle_increment);
    int right_10_index = round((-0.1745 - msg->angle_min) / msg->angle_increment);

    if (front_index < 0 || front_index >= msg->ranges.size() ||
        left_10_index < 0 || left_10_index >= msg->ranges.size() ||
        right_10_index < 0 || right_10_index >= msg->ranges.size())
    {
        ROS_WARN("Invalid indices for front or ±10 degrees.");
        return;
    }

    double front_distance = msg->ranges[front_index];
    double left_10_distance = msg->ranges[left_10_index];
    double right_10_distance = msg->ranges[right_10_index];

    if (front_distance < msg->range_min || front_distance > msg->range_max) front_distance = std::numeric_limits<double>::max();
    if (left_10_distance < msg->range_min || left_10_distance > msg->range_max) left_10_distance = std::numeric_limits<double>::max();
    if (right_10_distance < msg->range_min || right_10_distance > msg->range_max) right_10_distance = std::numeric_limits<double>::max();

    min_obstacle_distance_ = std::min({front_distance, left_10_distance, right_10_distance});

    // Apply correct speed adjustment based on LiDAR readings
    double new_speed = calculateSpeedFromDistance(min_obstacle_distance_);
    speed_limit_ = smoothSpeedTransition(new_speed);

    publishDistanceToObstacle(min_obstacle_distance_);
    publishSpeedLimit();

    // Update min_vel_x only when the global planner has provided a valid plan
    updateMinVelocity();

    ROS_INFO("Laser Callback Processed: Min Distance = %.2f, Speed Limit Updated = %.2f", min_obstacle_distance_, speed_limit_);
}



double SpeedFilterLayer::calculateSpeedFromDistance(double distance)
{
    double calculated_speed;
    if (distance < 0.1)
        calculated_speed = 0.2;
    else if (distance < 0.5)
        calculated_speed = 0.4;
    else
        calculated_speed = 0.6;

    ROS_INFO("Calculated Speed: %.2f for distance: %.2f", calculated_speed, distance);
    return calculated_speed;
}



//Function to gradually adjust speed changes
double SpeedFilterLayer::smoothSpeedTransition(double new_speed)
{
    double alpha = 0.3; 
    return alpha * new_speed + (1 - alpha) * speed_limit_;
}


void SpeedFilterLayer::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (!isLocalPlanValid())  // ❌ Skip if no valid local plan is available
    {
        ROS_WARN_THROTTLE(1.0, "Skipping cmd_vel adjustment - No valid local plan available.");
        return;
    }

    if (!msg)  // ✅ Ensure message exists
    {
        ROS_WARN("Received NULL cmd_vel message.");
        return;
    }

    geometry_msgs::Twist new_vel = *msg;

    // ✅ Only adjust speed if necessary
    if (msg->linear.x > speed_limit_)
    {
        new_vel.linear.x = std::max(speed_limit_, 0.0);  
    }

    new_vel.angular.z = msg->angular.z;

    // ✅ Ensure smooth transitions
    new_vel.linear.x = smoothSpeedTransition(new_vel.linear.x);

    // ✅ Publish adjusted velocity
    vel_pub_.publish(new_vel);
    ROS_INFO_THROTTLE(1.0, "Published Adjusted Velocity to /cmd_vel_filtered: %.2f (Speed Limit: %.2f)", new_vel.linear.x, speed_limit_);
}



void SpeedFilterLayer::publishSpeedLimit()
{
    std_msgs::Float32 msg;
    msg.data = speed_limit_;
    speed_pub_.publish(msg);
}

void SpeedFilterLayer::publishDistanceToObstacle(double distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    distance_pub_.publish(msg);
}

}  // namespace speed_filter_layer

