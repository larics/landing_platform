cmake_minimum_required(VERSION 3.0.2)
project(landing_platform)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  message_generation
  tf
  uav_ros_lib
)
find_package(cmake_modules REQUIRED)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package()

include_directories(
  include()
  ${catkin_INCLUDE_DIRS}
)

add_library(landing_platform 
    src/landing_platform.cpp)
target_link_libraries(landing_platform ${catkin_LIBRARIES})

add_executable(landing_platform_node nodes/LandingPlatformNode.cpp)
target_link_libraries(landing_platform_node
      ${catkin_LIBRARIES} landing_platform)

add_executable(tf_node src/tf2_broadcaster.cpp)
target_link_libraries(tf_node
    ${catkin_LIBRARIES}
)