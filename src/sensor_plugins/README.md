# Sensors plugins for Gazebo

## Rangefinder plugin

Modified version from the `gazebo_plugins:` [gazebo_ros_range.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_range.cpp "Github page")

### Description
- Simulate rangefinder sensor in Gazebo
- The original plugin has been extended to publish tranform message on topic `/tf_gazebo_static`. To make this transformation visible in ROS use our [Static transform republisher plugin](../world_plugins/README.md#static-transform-republisher-plugin) in your `world` definition. 


### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <plugin name='mrs_gazebo_rangefinder' filename='libMRSGazeboRangefinderPlugin.so'>
      <updateRate>${frame_rate}</updateRate>
      <topicName>${topic}</topicName>
      <parentFrameName>${parent_frame_name}</parentFrameName>
      <frameName>${rangefinder_frame_name}</frameName>
      <gaussianNoise>${noise}</gaussianNoise>
      <fov>${fov}</fov>
      <radiation>${radiation}</radiation>
      <alwaysOn>true</alwaysOn>
      <x>${x}</x>
      <y>${y}</y>
      <z>${z}</z>
      <roll>${roll}</roll>
      <pitch>${pitch}</pitch>
      <yaw>${yaw}</yaw>
    </plugin>
  ...
```

## 2D lidar plugin

Modified version from the `gazebo_plugins:` [gazebo_ros_laser.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_laser.cpp "Github page")

### Description
- Simulate 2D lidar sensor in Gazebo
- The original plugin has been extended to publish tranform message on topic `/tf_gazebo_static`. To make this transformation visible in ROS use our [Static transform republisher plugin](../world_plugins/README.md#static-transform-republisher-plugin) in your `world` definition. 


### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <plugin name='mrs_gazebo_2dlidar' filename='libMRSGazebo2DLidarPlugin.so'>
      <topicName>${topic_name}</topicName>
      <frameName>${frame_name}</frameName>
      <parentFrameName>${parent_frame_name}</parentFrameName>
      <x>${x}</x>
      <y>${y}</y>
      <z>${z}</z>
      <roll>${roll}</roll>
      <pitch>${pitch}</pitch>
      <yaw>${yaw}</yaw>
    </plugin>
  ...
```

## GPS plugin

Modified version from the `sitl_gazebo:` [gazebo_gps_plugin.cpp](https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_gps_plugin.cpp "Github page")

### Description
- Publishes GPS and Groundtruth data using gazebo msgs for the PX4 firmware.
- The home position can be specified using the environment variables `PX4_HOME_LAT`, `PX4_HOME_LON`, and `PX4_HOME_ALT`. 

  ```bash
  ## For example: Zurich Irchel Park gps coordinates = '47.397742° N, 8.545594° E, and 488 m altitude'
  PI=$(echo "scale=10; 4*a(1)" | bc -l)
  export PX4_HOME_LAT=$((47.397742 * PI / 180.0))  # rad
  export PX4_HOME_LON=$((8.545594 * PI / 180.0))   # rad
  export PX4_HOME_ALT=488.0;                       # meters
  ```

- The original plugin has been extended to enable gps jamming inside of building (parameter `${gps_indoor_jamming}` has to be boolean).
 
### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <plugin name="gps_plugin" filename="libMRSGazeboGPSPlugin.so">
      <robotNamespace>${namespace}</robotNamespace>
      <gpsNoise>${gps_noise}</gpsNoise>
      <indoorJamming>${gps_indoor_jamming}</indoorJamming>
    </plugin>
  ...
```
