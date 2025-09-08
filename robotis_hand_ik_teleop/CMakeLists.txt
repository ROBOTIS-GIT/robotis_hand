################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robotis_hand_ik_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(urdf REQUIRED)

################################################################################
# Build
################################################################################
add_executable(robotis_hand_ik_teleop src/hand_ik_teleop.cpp src/position_only_ik_vel_solver.cpp)

target_include_directories(robotis_hand_ik_teleop PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/robotis_hand_ik_teleop>
)

target_link_libraries(robotis_hand_ik_teleop PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${kdl_parser_TARGETS}
  ${orocos_kdl_vendor_TARGETS}
  ${urdf_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  robotis_hand_ik_teleop
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
