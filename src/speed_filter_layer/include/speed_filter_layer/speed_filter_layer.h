#ifndef SPEED_FILTER_LAYER_H_
#define SPEED_FILTER_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <ros/subscriber.h>

namespace speed_filter_layer
{
class SpeedFilterLayer : public costmap_2d::Layer
{
public:
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j);

private:
    ros::Subscriber cmd_vel_sub_;  
    ros::Publisher vel_pub_;
    double resolution_, origin_x_, origin_y_;
    unsigned int map_width_, map_height_;
    geometry_msgs::Twist last_cmd_vel_; 
    double occupied_thresh;
    double free_thresh;
    double getSpeedFactor(double x, double y);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    cv::Mat speed_map_;

};
}
#endif

