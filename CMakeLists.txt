cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_ENPM808X)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs move_base_msgs sensor_msgs)

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(roomba src/main.cpp src/bot.cpp)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roomba ${catkin_LIBRARIES})
