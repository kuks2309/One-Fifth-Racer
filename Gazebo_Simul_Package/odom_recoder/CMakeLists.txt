cmake_minimum_required(VERSION 3.0.2)
project(odom_recorder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES odom_recoder
   CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_recoder_node src/odom_recoder.cpp)
add_dependencies(odom_recoder_node ${odom_recoder_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_recoder_node ${catkin_LIBRARIES} )

add_executable(odom_display_node src/odom_display.cpp)
add_dependencies(odom_recoder_node ${odom_display_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_display_node ${catkin_LIBRARIES} )


