cmake_minimum_required(VERSION 3.0.2)
project(m_car_encoder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_geometry_msgs
  serial
)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msg geometry_msgs tf2_geometry_msgs sensor_msgs tf serial
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

#add_executable(utm_odom_node src/gps_odom.cpp)
#add_dependencies(utm_odom_node ${utm_odom_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(utm_odom_node ${catkin_LIBRARIES} )

add_executable(henes_car_control_node src/henes_car_control_node.cpp)
add_dependencies(henes_car_control_node ${henes_car_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(henes_car_control_node ${catkin_LIBRARIES} )

add_executable(rosbag_file_data_save src/rosbag_file_save.cpp)
add_dependencies(rosbag_file_data_save ${rosbag_file_data_save_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_file_data_save ${catkin_LIBRARIES})
