cmake_minimum_required(VERSION 3.5)
project(mrs_gazebo_common_resources)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

set(CATKIN_DEPENDENCIES
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# include Gazebo
find_package(gazebo REQUIRED)

# include Qt
find_package(Qt5Widgets REQUIRED)

# include OpenCV
find_package(OpenCV REQUIRED)

# include Boost
find_package(Boost REQUIRED COMPONENTS thread)

# include Eigen3
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
find_package(Protobuf REQUIRED)

set(sensor_msgs
  msgs/SITLGps.proto
  msgs/Groundtruth.proto
  msgs/MagneticField.proto
  msgs/Float.proto
  msgs/CommandMotorSpeed.proto
  msgs/MotorSpeed.proto
  msgs/Wind.proto
  )

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###################################
## catkin specific configuration ##
###################################

set(LIBRARIES
  MrsGazeboCommonResources_GuiManager
  MrsGazeboCommonResources_RvizCameraSynchronizer
  MrsGazeboCommonResources_StaticTransformRepublisher
  MrsGazeboCommonResources_RangefinderPlugin
  MrsGazeboCommonResources_2DLidarPlugin
  MrsGazeboCommonResources_3DLidarPlugin
  MrsGazeboCommonResources_3DLidarGpuPlugin
  MrsGazeboCommonResources_CameraPlugin
  MrsGazeboCommonResources_RealsensePlugin
  MrsGazeboCommonResources_ParachutePlugin
  MrsGazeboCommonResources_WaterGunPlugin
  MrsGazeboCommonResources_GPSPlugin
  MrsGazeboCommonResources_MagnetometerPlugin
  MrsGazeboCommonResources_LightPlugin
  MrsGazeboCommonResources_ServoCameraPlugin
  MrsGazeboCommonResources_DynamicModelPlugin
  MrsGazeboCommonResources_SafetyLedPlugin
  MrsGazeboCommonResources_FluidResistancePlugin
  MrsGazeboCommonResources_BatteryPlugin
  MrsGazeboCommonResources_MotorSpeedRepublisherPlugin
  MrsGazeboCommonResources_MotorPropModelPlugin
  MrsGazeboCommonResources_LinkStaticTFPublisher
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS GAZEBO Qt5Widgets Boost Eigen OpenCV
  LIBRARIES ${LIBRARIES}
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src/sensor_and_model_plugins/propulsion_module/include
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

##  GUI plugins

add_library(MrsGazeboCommonResources_GuiManager SHARED
  src/gui_plugins/gui_manager.cpp
  )

target_link_libraries(MrsGazeboCommonResources_GuiManager
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  )

##  world plugins

add_library(MrsGazeboCommonResources_RvizCameraSynchronizer SHARED
  src/world_plugins/rviz_cam_synchronizer.cpp
  )

target_link_libraries(MrsGazeboCommonResources_RvizCameraSynchronizer
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  )

add_library(MrsGazeboCommonResources_StaticTransformRepublisher SHARED
  src/world_plugins/static_transform_republisher.cpp
  )

target_link_libraries(MrsGazeboCommonResources_StaticTransformRepublisher
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  )

##  sensor and model plugins

add_library(MrsGazeboCommonResources_RangefinderPlugin SHARED
  src/sensor_and_model_plugins/rangefinder_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_RangefinderPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  RayPlugin
  )

add_library(MrsGazeboCommonResources_2DLidarPlugin SHARED
  src/sensor_and_model_plugins/2dlidar_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_2DLidarPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  RayPlugin
  )

add_library(MrsGazeboCommonResources_3DLidarPlugin SHARED
  src/sensor_and_model_plugins/3dlidar_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_3DLidarPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  RayPlugin
  )

add_library(MrsGazeboCommonResources_3DLidarGpuPlugin SHARED
  src/sensor_and_model_plugins/3dlidar_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_3DLidarGpuPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  GpuRayPlugin
  )

target_compile_definitions(MrsGazeboCommonResources_3DLidarGpuPlugin PRIVATE GAZEBO_GPU_RAY=1)

add_library(MrsGazeboCommonResources_CameraPlugin SHARED
  src/sensor_and_model_plugins/camera_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_CameraPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  CameraPlugin
  )

# RealsensePlugin

add_library(MrsGazeboCommonResources_RealsensePlugin SHARED
  src/sensor_and_model_plugins/realsense_plugin.cpp
  src/common/perlin_noise.cpp
  )

target_link_libraries(MrsGazeboCommonResources_RealsensePlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

# ParachutePlugin

add_library(MrsGazeboCommonResources_ParachutePlugin SHARED
  src/sensor_and_model_plugins/parachute_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_ParachutePlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# WaterGunPlugin

add_library(MrsGazeboCommonResources_WaterGunPlugin SHARED
  src/sensor_and_model_plugins/water_gun_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_WaterGunPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# GPSPlugin

add_library(MrsGazeboCommonResources_GPSPlugin SHARED
  src/sensor_and_model_plugins/gps_plugin.cpp
  ${SEN_PROTO_SRCS}
  )

target_link_libraries(MrsGazeboCommonResources_GPSPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# MagnetometerPlugin

add_library(MrsGazeboCommonResources_MagnetometerPlugin SHARED
  src/sensor_and_model_plugins/magnetometer_plugin.cpp
  src/common/geo_mag_declination.cpp ${SEN_PROTO_SRCS}
  )

target_link_libraries(MrsGazeboCommonResources_MagnetometerPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# LightPlugin

add_library(MrsGazeboCommonResources_LightPlugin SHARED
  src/sensor_and_model_plugins/light_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_LightPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# ServoCameraPlugin

add_library(MrsGazeboCommonResources_ServoCameraPlugin SHARED
  src/sensor_and_model_plugins/servo_camera_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_ServoCameraPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# DynamicModelPlugin

add_library(MrsGazeboCommonResources_DynamicModelPlugin SHARED
  src/sensor_and_model_plugins/dynamic_model_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_DynamicModelPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# SafetyLedPlugin

add_library(MrsGazeboCommonResources_SafetyLedPlugin SHARED
  src/sensor_and_model_plugins/safety_led_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_SafetyLedPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# FluidResistancePlugin

add_library(MrsGazeboCommonResources_FluidResistancePlugin SHARED
  src/physics_plugins/fluid_resistance_plugin.cpp
  )

target_link_libraries(MrsGazeboCommonResources_FluidResistancePlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MrsGazeboCommonResources_BatteryPlugin SHARED
  src/sensor_and_model_plugins/battery_plugin.cpp
  src/sensor_and_model_plugins/propulsion_module/src/propulsion_module.cpp
  )

target_link_libraries(MrsGazeboCommonResources_BatteryPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
  )

# MotorSpeedRepublisherPlugin

add_library(MrsGazeboCommonResources_MotorSpeedRepublisherPlugin SHARED
  src/sensor_and_model_plugins/motor_speed_republisher.cpp
  ${SEN_PROTO_SRCS}
  )

target_link_libraries(MrsGazeboCommonResources_MotorSpeedRepublisherPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# MotorPropModelPlugin

add_library(MrsGazeboCommonResources_MotorPropModelPlugin SHARED
  src/physics_plugins/motor_prop_model_plugin.cpp
  ${SEN_PROTO_SRCS}
  )

target_link_libraries(MrsGazeboCommonResources_MotorPropModelPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

# LinkStaticTFPublisher

add_library(MrsGazeboCommonResources_LinkStaticTFPublisher SHARED
  src/sensor_and_model_plugins/link_static_tf_publisher.cpp
  )

target_link_libraries(MrsGazeboCommonResources_LinkStaticTFPublisher
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  )

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  )
