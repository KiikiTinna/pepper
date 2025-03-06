#ifndef RIGHT_SIDE_LAYER_H_
#define RIGHT_SIDE_LAYER_H_

#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

namespace right_side_layer_namespace
{

class RightSideLayer : public costmap_2d::CostmapLayer
{
public:
  RightSideLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j);
  virtual void reset();

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    // Configuration parameters
    double right_side_bias_factor_;
    double obstacle_detection_range_;
    
    // Robot pose tracking
    geometry_msgs::PoseWithCovarianceStamped current_pose_;
    bool pose_received_;
};

} // namespace right_side_layer_namespace

#endif // RIGHT_SIDE_LAYER_H_

