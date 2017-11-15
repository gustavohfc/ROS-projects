cmake_minimum_required(VERSION 2.8.3)
project(fcr2017)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  p2os_msgs
  tf
  laser_line_extraction
)

catkin_package(
 # INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_sonar src/gazebo_sonar.cpp)
target_link_libraries(gazebo_sonar ${catkin_LIBRARIES})

add_executable(publisher3at src/publisher3at.cc)
target_link_libraries(publisher3at ${catkin_LIBRARIES})

add_executable(pose src/pose.cpp)
target_link_libraries(pose ${catkin_LIBRARIES})

add_executable(forward_kinematics_pioneer src/forward_kinematics_pioneer.cpp)
target_link_libraries(forward_kinematics_pioneer ${catkin_LIBRARIES})

#add_executable(pwm_quad src/pwm_quad.cpp)
#target_link_libraries(pwm_quad ${catkin_LIBRARIES})

# Trabalho 3
add_executable(140021671_trabalho3
         src/140021671_trabalho3.cpp
         src/common_lib/laser_sensor.cpp
         src/common_lib/odometer.cpp
         src/common_lib/user_motion_controller.cpp
         src/common_lib/graph.cpp
         src/common_lib/grid_map.cpp
         src/common_lib/common.cpp)
target_link_libraries(140021671_trabalho3 ${catkin_LIBRARIES})