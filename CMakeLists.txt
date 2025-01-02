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
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts
)

# Executable 생성
add_executable(mpc_controller 
  src/main.cpp 
  src/Powerpack.cpp 
  src/DatabaseConfig.cpp
  src/PWM.cpp 
  src/ReferenceGoverner.cpp 
  src/Sensor.cpp 
  src/Solver.cpp
  src/Dynamics.cpp
  src/QP.cpp
)

add_executable(pid_controller 
  src/PosNegPIDController.cpp
  src/DatabaseConfig.cpp
  )
  
add_executable(data_publisher src/data_publisher.cpp)

# YAML-CPP 및 catkin 라이브러리 연결
find_package(YAML-CPP REQUIRED)

# target_link_libraries 수정 (일관된 방식 사용)
target_link_libraries(mpc_controller PRIVATE yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(pid_controller PRIVATE yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(data_publisher PRIVATE ${catkin_LIBRARIES})

# OSQP 라이브러리 연결
find_package(osqp REQUIRED)
target_link_libraries(mpc_controller PRIVATE osqp::osqp)

