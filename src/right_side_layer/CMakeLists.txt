cmake_minimum_required(VERSION 3.0.2)
project(right_side_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  pluginlib
  roscpp
  dynamic_reconfigure
  geometry_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES right_side_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib dynamic_reconfigure geometry_msgs tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(right_side_layer src/right_side_layer.cpp)

target_link_libraries(right_side_layer ${catkin_LIBRARIES})


target_include_directories(right_side_layer PUBLIC
  ${catkin_INCLUDE_DIRS}
)


# Install targets
install(TARGETS right_side_layer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install include headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install plugin description file
install(FILES right_side_layer_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

