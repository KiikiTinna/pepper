#ifndef SPEED_FILTER_LAYER_H_
#define SPEED_FILTER_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>  
#include <costmap_2d/layered_costmap.h>  
#include <sensor_msgs/LaserScan.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>


namespace speed_filter_layer
{
class SpeedFilterLayer : public costmap_2d::Layer
{
public:
    SpeedFilterLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                              double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    
private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    double calculateSpeedFromDistance(double distance);  
    void publishSpeedLimit();
    void publishDistanceToObstacle(double distance);
    double smoothSpeedTransition(double new_speed);
    void updateMinVelocity();
    void localPlanCallback(const nav_msgs::Path::ConstPtr& msg);


    ros::NodeHandle nh_;
    ros::Publisher speed_pub_;
    ros::Publisher distance_pub_;
    ros::Publisher vel_pub_;
    ros::Subscriber laser_sub_; 
    ros::Subscriber speed_sub_; 
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber local_plan_sub_;


    double speed_limit_;
    double slow_down_zone_radius_;
    double min_obstacle_distance_;  
    bool is_local_plan_received_;
    ros::Time last_local_plan_time_;
    bool isLocalPlanValid();
};
}  // namespace speed_filter_layer

#endif  // SPEED_FILTER_LAYER_H_

