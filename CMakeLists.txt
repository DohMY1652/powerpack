cmake_minimum_required(VERSION 3.0.2)
project(powerpack)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(Pos_neg_controller src/main.cpp src/Powerpack.cpp src/DatabaseConfig.cpp src/Controller.cpp src/ControlModule.cpp src/PWM.cpp src/ReferenceGoverner.cpp src/Sensor.cpp)

# YAML 라이브러리 링크
target_link_libraries(Pos_neg_controller PRIVATE yaml-cpp ${catkin_LIBRARIES})

# config.yaml 파일 설치
install(FILES config/config.yaml DESTINATION ${CMAKE_INSTALL_PREFIX}/config)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
