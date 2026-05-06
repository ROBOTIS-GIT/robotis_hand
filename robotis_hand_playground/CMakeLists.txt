cmake_minimum_required(VERSION 3.8)
project(robotis_hand_playground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robotis_interfaces REQUIRED)
set(ROS2_PACKAGES
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    control_msgs
    sensor_msgs
    trajectory_msgs
    robotis_interfaces)

add_executable(tactile_rviz
              src/tactile_rviz.cpp)
target_include_directories(tactile_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tactile_rviz ${ROS2_PACKAGES})
install(TARGETS tactile_rviz
        DESTINATION lib/${PROJECT_NAME})

add_executable(tactile_force_controller
              src/tactile_force_controller.cpp
              src/tactile_sensor.cpp
              src/hx5d20_init.cpp
              src/param.cpp)
target_include_directories(tactile_force_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tactile_force_controller ${ROS2_PACKAGES})
install(TARGETS tactile_force_controller
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/grasp_start_publisher.py
  scripts/matplot_visualize_tactile.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
