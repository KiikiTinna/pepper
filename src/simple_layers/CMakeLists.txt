cmake_minimum_required(VERSION 3.0.2)
project(simple_layers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  pluginlib
  roscpp
  nav_msgs
  sensor_msgs	
)


catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES simple_layers
    CATKIN_DEPENDS costmap_2d dynamic_reconfigure pluginlib roscpp nav_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(simple_layers
   src/simple_layer.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(simple_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(simple_layers ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


install(FILES costmap_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

 install(TARGETS simple_layers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )


