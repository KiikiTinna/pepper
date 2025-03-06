#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d.h>
#include <opencv2/core/core.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace simple_layers {

class SimpleLayer : public costmap_2d::Layer {
public:
  SimpleLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                           double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, 
                          int min_i, int min_j, int max_i, int max_j);

private:

  bool enabled_;
  // Method to publish custom costs for debugging
  void publishCustomCosts();
  
  // Method to publish a test grid for debugging
  void publishTestGrid();

  cv::Mat costly_map_;
  std::vector<std::vector<unsigned char>> custom_map_;
  double resolution_;
  double origin_x_, origin_y_;
  double occupied_thresh_, free_thresh_;
  int map_width_, map_height_;
  unsigned char avoidance_cost_;
  
  // Publisher for debugging
  ros::Publisher custom_cost_pub_;
};

} // namespace simple_layers

#endif // SIMPLE_LAYER_H_
