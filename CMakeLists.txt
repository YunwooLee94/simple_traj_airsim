cmake_minimum_required(VERSION 3.0.2)
project(simple_traj_airsim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf_conversions
  pcl_ros
)
add_service_files(FILES KeyboardInput.srv)
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        nav_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


catkin_package(
        LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
)


add_executable(teleop_drone src/keyboard_operator.cpp)
target_link_libraries(teleop_drone ${catkin_LIBRARIES} )


add_executable(simple_traj_airsim_node src/main.cpp)
target_link_libraries(simple_traj_airsim_node ${catkin_LIBRARIES})



