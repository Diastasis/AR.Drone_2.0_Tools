cmake_minimum_required(VERSION 2.8.3)
project(ardrone_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)

catkin_package()

include_directories(  ${catkin_INCLUDE_DIRS} )

add_executable(ardrone_teleop src/main.cpp)
target_link_libraries(ardrone_teleop ${catkin_LIBRARIES})

