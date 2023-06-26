cmake_minimum_required(VERSION 3.0.2)
project(icp_ros)

include(CheckIncludeFile)
include(CheckSymbolExists)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_geometry
  nav_msgs
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  tf2_ros
)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES icp_ros
  CATKIN_DEPENDS geometry_msgs laser_geometry nav_msgs roscpp sensor_msgs tf tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/icp.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)