cmake_minimum_required(VERSION 3.0.2)
project(powerpack)

# 수동으로 Eigen3 경로 지정
set(Eigen3_INCLUDE_DIR "/usr/include/eigen3")
include_directories(${Eigen3_INCLUDE_DIR})

# OSQP 경로 추가
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../osqp/include/public)

# 나머지 설정들
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(Pos_neg_controller 
  src/main.cpp 
  src/Powerpack.cpp 
  src/DatabaseConfig.cpp 
  src/Controller.cpp 
  src/ControlModule.cpp 
  src/PWM.cpp 
  src/ReferenceGoverner.cpp 
  src/Sensor.cpp 
  src/Valve.cpp
)

target_link_libraries(Pos_neg_controller PRIVATE yaml-cpp ${catkin_LIBRARIES})

# Find OSQP library and headers
find_package(osqp REQUIRED)

# Link the OSQP shared library
target_link_libraries(Pos_neg_controller PRIVATE osqp::osqp)
