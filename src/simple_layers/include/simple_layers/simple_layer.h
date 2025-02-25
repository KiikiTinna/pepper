#ifndef SIMPLE_LAYER_H
#define SIMPLE_LAYER_H

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace simple_layer_namespace
{
class SimpleLayer : public costmap_2d::Layer
{
public:
    SimpleLayer();
    void onInitialize() override;
    void updateBounds(double origin_x, double origin_y, double origin_yaw,
                      double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
    ros::Subscriber laser_sub_;
    ros::Subscriber goal_status_sub_;
    ros::Subscriber global_plan_sub_;

    ros::Publisher speed_pub_;
    ros::Timer update_timer_;

    bool goal_received_;
    bool goal_reached_;
    bool enabled_;

    double min_obstacle_distance_;
    double min_speed_;
    double max_speed_;
    double obstacle_distance_threshold_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg);
    void adjustSpeed();
    void updateMap(const ros::TimerEvent& event);
};
} // namespace simple_layer_namespace

#endif // SIMPLE_LAYER_H

