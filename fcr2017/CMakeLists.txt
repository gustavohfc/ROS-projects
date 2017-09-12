cmake_minimum_required(VERSION 2.8.3)
project(fcr2017)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  p2os_msgs
  tf
)

catkin_package(
 # INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf
)

include_directories(
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




# Roteiro 1
add_executable(140021671_direta src/Roteiro1/140021671_direta.cpp)
target_link_libraries(140021671_direta ${catkin_LIBRARIES})

add_executable(140021671_inversa src/Roteiro1/140021671_inversa.cpp)
target_link_libraries(140021671_inversa ${catkin_LIBRARIES})


# Roteiro 2
add_executable(140021671_sensor src/Roteiro2/140021671_sensor.cpp)
target_link_libraries(140021671_sensor ${catkin_LIBRARIES})


# Roteiro 3
add_executable(140021671_localizacao src/Roteiro3/140021671_localizacao.cpp)
target_link_libraries(140021671_localizacao ${catkin_LIBRARIES})
