#ifndef TEST_LAYER_H_
#define TEST_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

namespace test_layer
{
  
class TestLayer : public costmap_2d::CostmapLayer
{
public:
    TestLayer();
  
    virtual void onInitialize();
  
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);
  
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j);
  
    virtual void reset();

private:
    double min_x_, min_y_, max_x_, max_y_;
    unsigned char cost_value_;
    bool enabled_;
};

} // namespace test_layer

#endif  // TEST_LAYER_H_

