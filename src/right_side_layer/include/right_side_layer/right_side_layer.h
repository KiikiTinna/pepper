#ifndef RIGHT_SIDE_LAYER_H_
#define RIGHT_SIDE_LAYER_H_

#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

namespace right_side_layer
{

class RightSideLayer : public costmap_2d::CostmapLayer
{
public:
    RightSideLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                              double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

//  virtual void reset();

private:
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void resetLayer();
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    geometry_msgs::PoseWithCovarianceStamped initial_pose_;
    geometry_msgs::PoseStamped goal_pose_;

    bool initial_pose_received_;
    bool goal_received_;
    bool bias_applied_;

    double bias_factor_;
    double inflation_radius_;
    double goal_direction_;  // Angle from start to goal
    double outer_radius_;
};

} // namespace right_side_layer

#endif // RIGHT_SIDE_LAYER_H_

