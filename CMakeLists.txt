cmake_minimum_required(VERSION 3.2.0)
project(mrs_gazebo_common_resources)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  std_msgs
  gazebo_plugins
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
  )

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs std_msgs tf2_msgs gazebo_plugins
   DEPENDS GAZEBO Qt5Widgets Boost Eigen OpenCV
   LIBRARIES MRSGazeboGuiManager MRSGazeboRvizCameraSynchronizer MRSGazeboStaticTransformRepublisher MRSGazeboRangefinderPlugin MRSGazebo2DLidarPlugin MRSGazebo3DLidarPlugin MRSGazebo3DLidarGpuPlugin MRSGazeboCameraPlugin MRSGazeboRealsensePlugin MRSGazeboParachutePlugin MRSGazeboWaterGunPlugin MRSGazeboGPSPlugin MRSGazeboMagnetometerPlugin MRSGazeboLightPlugin MRSGazeboServoCameraPlugin MRSGazeboDynamicModelPlugin MRSGazeboMotorSpeedRepublisherPlugin
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
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
add_library(MRSGazeboGuiManager SHARED src/gui_plugins/gui_manager.cpp)
target_link_libraries(MRSGazeboGuiManager
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  )

##  world plugins
add_library(MRSGazeboRvizCameraSynchronizer SHARED src/world_plugins/rviz_cam_synchronizer.cpp)
target_link_libraries(MRSGazeboRvizCameraSynchronizer
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  )

add_library(MRSGazeboStaticTransformRepublisher SHARED src/world_plugins/static_transform_republisher.cpp)
target_link_libraries(MRSGazeboStaticTransformRepublisher
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  )

##  sensor and model plugins
add_library(MRSGazeboRangefinderPlugin SHARED src/sensor_and_model_plugins/rangefinder_plugin.cpp)
target_link_libraries(MRSGazeboRangefinderPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  RayPlugin
  )

add_library(MRSGazebo2DLidarPlugin SHARED src/sensor_and_model_plugins/2dlidar_plugin.cpp)
target_link_libraries(MRSGazebo2DLidarPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  RayPlugin
  )

add_library(MRSGazebo3DLidarPlugin SHARED src/sensor_and_model_plugins/3dlidar_plugin.cpp)
target_link_libraries(MRSGazebo3DLidarPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  RayPlugin
  )

add_library(MRSGazebo3DLidarGpuPlugin SHARED src/sensor_and_model_plugins/3dlidar_plugin.cpp)
target_link_libraries(MRSGazebo3DLidarGpuPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  GpuRayPlugin
  )
target_compile_definitions(MRSGazebo3DLidarGpuPlugin PRIVATE GAZEBO_GPU_RAY=1)

add_library(MRSGazeboCameraPlugin SHARED src/sensor_and_model_plugins/camera_plugin.cpp)
target_link_libraries(MRSGazeboCameraPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  CameraPlugin
  )

add_library(MRSGazeboRealsensePlugin SHARED src/sensor_and_model_plugins/realsense_plugin.cpp src/common/perlin_noise.cpp)
target_link_libraries(MRSGazeboRealsensePlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_library(MRSGazeboParachutePlugin SHARED src/sensor_and_model_plugins/parachute_plugin.cpp)
target_link_libraries(MRSGazeboParachutePlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSGazeboWaterGunPlugin SHARED src/sensor_and_model_plugins/water_gun_plugin.cpp)
target_link_libraries(MRSGazeboWaterGunPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSGazeboGPSPlugin SHARED src/sensor_and_model_plugins/gps_plugin.cpp ${SEN_PROTO_SRCS})
target_link_libraries(MRSGazeboGPSPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSGazeboMagnetometerPlugin SHARED src/sensor_and_model_plugins/magnetometer_plugin.cpp src/common/geo_mag_declination.cpp ${SEN_PROTO_SRCS})
target_link_libraries(MRSGazeboMagnetometerPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

##  model plugins
add_library(MRSGazeboLightPlugin SHARED src/sensor_and_model_plugins/light_plugin.cpp)
target_link_libraries(MRSGazeboLightPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSGazeboServoCameraPlugin SHARED src/sensor_and_model_plugins/servo_camera_plugin.cpp)
target_link_libraries(MRSGazeboServoCameraPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSGazeboDynamicModelPlugin SHARED src/sensor_and_model_plugins/dynamic_model_plugin.cpp)
target_link_libraries(MRSGazeboDynamicModelPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSSafetyLedPlugin SHARED src/sensor_and_model_plugins/safety_led_plugin.cpp)
target_link_libraries(MRSSafetyLedPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_library(MRSGazeboMotorSpeedRepublisherPlugin SHARED src/sensor_and_model_plugins/motor_speed_republisher.cpp)
target_link_libraries(MRSGazeboMotorSpeedRepublisherPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS  MRSGazeboGuiManager MRSGazeboRvizCameraSynchronizer MRSGazeboStaticTransformRepublisher MRSGazeboRangefinderPlugin MRSGazebo2DLidarPlugin MRSGazebo3DLidarPlugin MRSGazebo3DLidarGpuPlugin MRSGazeboCameraPlugin MRSGazeboRealsensePlugin MRSGazeboParachutePlugin MRSGazeboWaterGunPlugin MRSGazeboGPSPlugin MRSGazeboMagnetometerPlugin MRSGazeboLightPlugin MRSGazeboServoCameraPlugin MRSGazeboDynamicModelPlugin MRSGazeboMotorSpeedRepublisherPlugin
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

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
