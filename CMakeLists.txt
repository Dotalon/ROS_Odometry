cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options( ##we make it compile the file for generation
  cfg/lidar_visualization_node.cfg
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES} armadillo)

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES} )

add_executable(lidar_visualization_node src/lidar_visualization_node.cpp)
add_dependencies(lidar_visualization_node ${PROJECT_NAME}_gencfg)
target_link_libraries(lidar_visualization_node ${catkin_LIBRARIES})

