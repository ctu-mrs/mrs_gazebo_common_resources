cmake_minimum_required(VERSION 3.5)
project(mrs_gazebo_common_resources)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_px4_api]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
  add_definitions("-g")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(DEPENDENCIES
  rclcpp
  ros_gz
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

include_directories(
  ${rclcpp_INCLUDE_DIRECTORIES}
  ${ros_gz_INCLUDE_DIRECTORIES}
  ${ros_gz_sim_INCLUDE_DIRECTORIES}
)


# # XXX this approach is extremely error prone
# # it would be preferable to either depend on the
# # compiled headers from Gazebo directly
# # or to have something entirely independent.
# #
# set(PROTOBUF_IMPORT_DIRS "")
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#   endif()
# endforeach()

# set(GAZEBO_MSG_INCLUDE_DIRS)
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
#   endif()
# endforeach()

# # PROTOBUF_IMPORT_DIRS has to be set before
# find_package(Protobuf REQUIRED)

# set(sensor_msgs
#   msgs/SITLGps.proto
#   msgs/Groundtruth.proto
#   msgs/MagneticField.proto
#   msgs/Float.proto
#   msgs/CommandMotorSpeed.proto
#   msgs/MotorSpeed.proto
#   msgs/Wind.proto
#   )

# PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

###################################
## catkin specific configuration ##
###################################

# set(LIBRARIES
#   MrsGazeboCommonResources_GuiManager
#   MrsGazeboCommonResources_RvizCameraSynchronizer
#   MrsGazeboCommonResources_StaticTransformRepublisher
#   MrsGazeboCommonResources_RangefinderPlugin
#   MrsGazeboCommonResources_2DLidarPlugin
#   MrsGazeboCommonResources_3DLidarPlugin
#   MrsGazeboCommonResources_3DLidarGpuPlugin
#   MrsGazeboCommonResources_CameraPlugin
#   MrsGazeboCommonResources_RealsensePlugin
#   MrsGazeboCommonResources_ParachutePlugin
#   MrsGazeboCommonResources_WaterGunPlugin
#   MrsGazeboCommonResources_GPSPlugin
#   MrsGazeboCommonResources_MagnetometerPlugin
#   MrsGazeboCommonResources_LightPlugin
#   MrsGazeboCommonResources_ServoCameraPlugin
#   MrsGazeboCommonResources_DynamicModelPlugin
#   MrsGazeboCommonResources_SafetyLedPlugin
#   MrsGazeboCommonResources_FluidResistancePlugin
#   MrsGazeboCommonResources_MotorSpeedRepublisherPlugin
#   MrsGazeboCommonResources_MotorPropModelPlugin
#   MrsGazeboCommonResources_LinkStaticTFPublisher
#   )

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
#   DEPENDS GAZEBO Qt5Widgets Boost Eigen OpenCV
#   LIBRARIES ${LIBRARIES}
#   )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${GAZEBO_INCLUDE_DIRS}
#   ${GAZEBO_MSG_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_BINARY_DIR}
#   ${catkin_INCLUDE_DIRS}
#   ${Qt5Widgets_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
#   ${Eigen_INCLUDE_DIRS}
#   )

# link_directories(${GAZEBO_LIBRARY_DIRS})

##  GUI plugins

# add_library(MrsGazeboCommonResources_GuiManager SHARED
#   src/gui_plugins/gui_manager.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_GuiManager
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Qt5Widgets_LIBRARIES}
#   )

##  world plugins

# add_library(MrsGazeboCommonResources_RvizCameraSynchronizer SHARED
#   src/world_plugins/rviz_cam_synchronizer.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_RvizCameraSynchronizer
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   )

# add_library(MrsGazeboCommonResources_StaticTransformRepublisher SHARED
#   src/world_plugins/static_transform_republisher.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_StaticTransformRepublisher
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   )

##  sensor and model plugins

# add_library(MrsGazeboCommonResources_RangefinderPlugin SHARED
#   src/sensor_and_model_plugins/rangefinder_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_RangefinderPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   RayPlugin
#   )

# add_library(MrsGazeboCommonResources_2DLidarPlugin SHARED
#   src/sensor_and_model_plugins/2dlidar_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_2DLidarPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   RayPlugin
#   )

# add_library(MrsGazeboCommonResources_3DLidarPlugin SHARED
#   src/sensor_and_model_plugins/3dlidar_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_3DLidarPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   RayPlugin
#   )

# add_library(MrsGazeboCommonResources_3DLidarGpuPlugin SHARED
#   src/sensor_and_model_plugins/3dlidar_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_3DLidarGpuPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   GpuRayPlugin
#   )

# target_compile_definitions(MrsGazeboCommonResources_3DLidarGpuPlugin PRIVATE GAZEBO_GPU_RAY=1)

# add_library(MrsGazeboCommonResources_CameraPlugin SHARED
#   src/sensor_and_model_plugins/camera_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_CameraPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   CameraPlugin
#   )

# RealsensePlugin

# add_library(MrsGazeboCommonResources_RealsensePlugin SHARED
#   src/sensor_and_model_plugins/realsense_plugin.cpp
#   src/common/perlin_noise.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_RealsensePlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   )

# ParachutePlugin

# add_library(MrsGazeboCommonResources_ParachutePlugin SHARED
#   src/sensor_and_model_plugins/parachute_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_ParachutePlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# WaterGunPlugin

# add_library(MrsGazeboCommonResources_WaterGunPlugin SHARED
#   src/sensor_and_model_plugins/water_gun_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_WaterGunPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# GPSPlugin

# add_library(MrsGazeboCommonResources_GPSPlugin SHARED
#   src/sensor_and_model_plugins/gps_plugin.cpp
#   ${SEN_PROTO_SRCS}
#   )

# target_link_libraries(MrsGazeboCommonResources_GPSPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# MagnetometerPlugin

# add_library(MrsGazeboCommonResources_MagnetometerPlugin SHARED
#   src/sensor_and_model_plugins/magnetometer_plugin.cpp
#   src/common/geo_mag_declination.cpp ${SEN_PROTO_SRCS}
#   )

# target_link_libraries(MrsGazeboCommonResources_MagnetometerPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# LightPlugin

# add_library(MrsGazeboCommonResources_LightPlugin SHARED
#   src/sensor_and_model_plugins/light_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_LightPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# ServoCameraPlugin

# add_library(MrsGazeboCommonResources_ServoCameraPlugin SHARED
#   src/sensor_and_model_plugins/servo_camera_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_ServoCameraPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# DynamicModelPlugin

# add_library(MrsGazeboCommonResources_DynamicModelPlugin SHARED
#   src/sensor_and_model_plugins/dynamic_model_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_DynamicModelPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# SafetyLedPlugin

# add_library(MrsGazeboCommonResources_SafetyLedPlugin SHARED
#   src/sensor_and_model_plugins/safety_led_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_SafetyLedPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# FluidResistancePlugin

# add_library(MrsGazeboCommonResources_FluidResistancePlugin SHARED
#   src/physics_plugins/fluid_resistance_plugin.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_FluidResistancePlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# MotorSpeedRepublisherPlugin

# add_library(MrsGazeboCommonResources_MotorSpeedRepublisherPlugin SHARED
#   src/sensor_and_model_plugins/motor_speed_republisher.cpp
#   ${SEN_PROTO_SRCS}
#   )

# target_link_libraries(MrsGazeboCommonResources_MotorSpeedRepublisherPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# MotorPropModelPlugin

# add_library(MrsGazeboCommonResources_MotorPropModelPlugin SHARED
#   src/physics_plugins/motor_prop_model_plugin.cpp
#   ${SEN_PROTO_SRCS}
#   )

# target_link_libraries(MrsGazeboCommonResources_MotorPropModelPlugin
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

# LinkStaticTFPublisher

# add_library(MrsGazeboCommonResources_LinkStaticTFPublisher SHARED
#   src/sensor_and_model_plugins/link_static_tf_publisher.cpp
#   )

# target_link_libraries(MrsGazeboCommonResources_LinkStaticTFPublisher
#   ${GAZEBO_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   )

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

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
