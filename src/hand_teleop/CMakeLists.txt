cmake_minimum_required(VERSION 3.8)

project(hand_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hand_interfaces REQUIRED)

add_executable(
  glove_node
  src/glove_node.cpp
  src/glove_driver.cpp
  src/glove_kinematics.cpp
  src/joint_mapper.cpp
)

target_include_directories(
  glove_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
  glove_node
  rclcpp
  hand_interfaces
)

install(
  TARGETS
    glove_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)

install(
  DIRECTORY
    config/
  DESTINATION
    share/${PROJECT_NAME}/config
)

ament_package()