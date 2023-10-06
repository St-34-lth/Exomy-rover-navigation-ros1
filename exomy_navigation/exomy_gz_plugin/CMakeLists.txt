cmake_minimum_required(VERSION 2.8.3)
project(exomy_gz_plugin)

add_definitions(-std=c++11)


# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_dev
  exomy_sim_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    gazebo_dev
    exomy_sim_msgs
)

add_library(exomy_gz_plugin src/exomy_rover_plugin.cpp)
target_link_libraries(exomy_gz_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})