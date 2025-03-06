cmake_minimum_required(VERSION 3.0.2)
project(speed_filter_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  dynamic_reconfigure
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES speed_filter_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib sensor_msgs geometry_msgs std_msgs dynamic_reconfigure nav_msgs 
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(speed_filter_layer
   src/speed_filter_layer.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
##${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
 target_link_libraries(speed_filter_layer 
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)


#############
## Install ##
#############

install(FILES speed_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )


 install(TARGETS speed_filter_layer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )









