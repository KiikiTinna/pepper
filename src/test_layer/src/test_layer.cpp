
#include "test_layer/test_layer.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


PLUGINLIB_EXPORT_CLASS(test_layer::TestLayer, costmap_2d::Layer)

namespace test_layer
{
TestLayer::TestLayer() : cost_value_(40), enabled_(true) {
}
void TestLayer::onInitialize()
{
    // Create a NodeHandle in the private namespace of this layer.
    ros::NodeHandle nh("~/" + name_);

    // Mark the layer as current and set the default costmap value.
    current_ = true;
    default_value_ = costmap_2d::FREE_SPACE;

    // Retrieve parameters for the rectangular zone from the parameter server.
    nh.param("min_x", min_x_, -2.0);
    nh.param("min_y", min_y_, -2.0);
    nh.param("max_x", max_x_, 2.0);
    nh.param("max_y", max_y_, 2.0);
    nh.param("cost_value", (int&)cost_value_, 40);
    nh.param("enabled", enabled_, true);

    // Ensure the layer's size matches the costmap.
    matchSize();
}

void TestLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                             double* min_x, double* min_y, double* max_x, double* max_y)
{
    // If the layer is disabled, do not update bounds.
    if (!enabled_)
        return;

    // Update the bounding box for the region where this layer is active.
    *min_x = std::min(*min_x, min_x_);
    *min_y = std::min(*min_y, min_y_);
    *max_x = std::max(*max_x, max_x_);
    *max_y = std::max(*max_y, max_y_);
}

void TestLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                            int min_i, int min_j, int max_i, int max_j)
{
    // If the layer is disabled, do not update costs.
    if (!enabled_)
        return;

    double wx, wy;
    // Iterate over the specified cell bounds.
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            // Convert grid indices to world coordinates.
            master_grid.mapToWorld(i, j, wx, wy);

            // Check if the world coordinate lies within the defined rectangular zone.
            if (wx >= min_x_ && wx <= max_x_ && wy >= min_y_ && wy <= max_y_) {
                unsigned char current_cost = master_grid.getCost(i, j);
                // Only update the cell if it isn't already marked as a lethal obstacle.
                if (current_cost < costmap_2d::LETHAL_OBSTACLE) {
                    master_grid.setCost(i, j, cost_value_);
                }
            }
        }
    }
}

void TestLayer::reset()
{
    // No dynamic state to reset in this simple layer.
}

}  // namespace test_layer


