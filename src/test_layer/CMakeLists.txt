cmake_minimum_required(VERSION 3.0.2)
project(test_layer)

add_compile_options(-std=c++11)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  roscpp
  pluginlib
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test_layer
  CATKIN_DEPENDS costmap_2d roscpp pluginlib
)

###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build the library
add_library(test_layer
  src/test_layer.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
##${catkin_EXPORTED_TARGETS})


target_link_libraries(test_layer
  ${catkin_LIBRARIES}
)

# Make sure the compiler can find the necessary headers
target_include_directories(test_layer PUBLIC
  ${catkin_INCLUDE_DIRS}
)

# Install targets
install(TARGETS test_layer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install include headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install plugin description file
install(FILES test_layer_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
