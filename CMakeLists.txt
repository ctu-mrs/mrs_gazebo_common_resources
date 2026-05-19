cmake_minimum_required(VERSION 3.5)
project(mrs_gazebo_common_resources)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_gazebo_common_resources]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

if ($ENV{ROS_DISTRO} STREQUAL "jazzy")
  set(GZ_VERSION "8")
else()
  message("Using unsupported ROS version! You have to MAKE MANUAL CHANGES in the CMakeLists.txt and ENSURE CORRECT ROS-GZ VERSION combination, e.g. jazzy + gz-sim8. See for reference: https://gazebosim.org/docs/latest/ros_installation/")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  gz-sim${GZ_VERSION}
  ros_gz_sim
  std_msgs
  geometry_msgs
  tf2
  tf2_msgs
  tf2_ros
  Eigen3
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(gz-plugin2 REQUIRED COMPONENTS register)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRECTORIES}
  ${ros_gz_INCLUDE_DIRECTORIES}
  ${ros_gz_sim_INCLUDE_DIRECTORIES}
)

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

add_library(MrsGazeboCommonResources_MulticopterMotorModel SHARED
  src/mrs_multicopter_motor_model.cpp
)

ament_target_dependencies(MrsGazeboCommonResources_MulticopterMotorModel
  rclcpp
)

target_link_libraries(MrsGazeboCommonResources_MulticopterMotorModel
  gz-sim${GZ_VERSION}
  gz-plugin2::register
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  MrsGazeboCommonResources_MulticopterMotorModel
)

ament_target_dependencies(MrsGazeboCommonResources_MulticopterMotorModel
  rclcpp
)

install(
  TARGETS MrsGazeboCommonResources_MulticopterMotorModel
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/extend_gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/extend_gz_sim_system_plugin_path.dsv.in")

ament_package()
