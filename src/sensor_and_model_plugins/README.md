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
- Simulates a 2D lidar sensor in Gazebo.
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

## 3D lidar plugin

Modified version of the canonical Velodyne/Ouster Gazebo plugins.

### Description
- Simulates a 3D lidar sensor in Gazebo.
- The original plugin has been extended to publish tranform message on topic `/tf_gazebo_static`. To make this transformation visible in ROS use our [Static transform republisher plugin](../world_plugins/README.md#static-transform-republisher-plugin) in your `world` definition.
- An IMU may be also enabled to simulate sensors such as the Ouster OSx with integrated IMU.
- Optimized the plugin to remove bottlenecks, causing RT factor drops.


### Usage
For example usage, see the Ouster section in `component_snippets.xacro`.

### Maintainer
For questions, pull-requests, bug reporting etc., please use the related Git page or contact the maintainer directly at vrbamato@fel.cvut.cz.

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

## Camera plugin

Modified version from the `gazebo_plugins:` [gazebo_ros_camera.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_camera.cpp "Github page")

### Description
- Simulates a camera sensor in Gazebo.
- The original plugin has been extended to publish tranform message on topic `/tf_gazebo_static`. To make this transformation visible in ROS use our [Static transform republisher plugin](../world_plugins/README.md#static-transform-republisher-plugin) in your `world` definition. 


### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <plugin name='mrs_gazebo_camera' filename='libMRSGazeboCameraPlugin.so'>
      <alwaysOn>true</alwaysOn>
      <updateRate>${frame_rate}</updateRate>
      <cameraName>${camera_suffix}</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>/${camera_frame_name}</frameName>
      <parentFrameName>${parent_frame_name}</parentFrameName>
      <sensorBaseFrameName>${sensor_base_frame_name}</sensorBaseFrameName>
      <x>${x}</x>
      <y>${y}</y>
      <z>${z}</z>
      <roll>${roll}</roll>
      <pitch>${pitch}</pitch>
      <yaw>${yaw}</yaw>
    </plugin>
  ...
```

## Realsense plugin

Modified version of the official [Realsense Gazebo plugin](https://github.com/intel/gazebo-realsense "Github page") from Intel

### Description
- Simulates a Realsense D435 RGB-D camera sensor in Gazebo.
- The original plugin has been extended to publish tranform message on topic `/tf_gazebo_static`. To make this transformation visible in ROS use our [Static transform republisher plugin](../world_plugins/README.md#static-transform-republisher-plugin) in your `world` definition. 
- A "realistic" mode with more noise and virtually reduced resolution may be activated using the `useRealistic` flag in the SDF.


### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <plugin name="mrs_gazebo_realsense" filename="libMRSGazeboRealsensePlugin.so">
      <useRealistic>${enable_realistic_realsense}</useRealistic>
      <noisePerMeter>0.2</noisePerMeter>
      <minNoiseDistance>4.0</minNoiseDistance>
      <perlinEmptyThreshold>0.8</perlinEmptyThreshold>
      <perlinEmptySpeed>0.2</perlinEmptySpeed>
      <imageScaling>4</imageScaling>
      <blurSize>15</blurSize>
      <erosionSize>5</erosionSize>
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

## Servo camera plugin

### Description
- Simulates camera with adjustable pitch angle

### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <joint name="${name}_joint" type="revolute">
        <parent link="${parent}"/>
        <child link="${name}_link"/>
        <axis xyz="0 1 0"/>
        <dynamics damping="0.05" friction="0.05"/>
        <limit upper="1.57" lower="-1.57" effort="1" velocity="10" />
        <origin xyz="${offset_x} ${offset_y} ${offset_z}" rpy="0.0 0.0 0.0"/>
    </joint>
    <link name="${name}_link">
        <inertial>
            <mass value="1e-3" />
            <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1e-5"/>
        </inertial>
      <visual>
        <geometry>
          <box size="0.01 0.01 0.01" />
        </geometry>
      </visual>
    </link>
    <gazebo>
      <plugin name="servo_camera_plugin" filename="libMRSGazeboServoCameraPlugin.so">
        <offset_x>${offset_x}</offset_x>
        <offset_y>${offset_y}</offset_y>
        <offset_z>${offset_z}</offset_z>
        <offset_pitch>${offset_pitch}</offset_pitch>
        <offset_yaw>${offset_yaw}</offset_yaw>
        <offset_roll>${offset_roll}</offset_roll>
        <spawning_frame>${parent}</spawning_frame>
        <update_rate>${update_rate}</update_rate>
        <max_pitch_rate>${max_pitch_rate}</max_pitch_rate>
        <joint_name>${name}_joint</joint_name>
        <camera_type>${camera_type}</camera_type>
      </plugin>
    </gazebo>
  ...
```

The angle can be set by publishing desired camera angle on topic /uav_name/servo_camera/set_pitch e.g. 
```
rostopic pub /uav1/servo_camera/set_pitch std_msgs/Float32 "data: 1.0"
```

The camera image is published on topic /servo_camera/image_raw.

## Light plugin

### Description
- Simulate light with adjustable pitch angle mounted on the robot's frame

### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <joint name="${name}_joint" type="revolute">
        <parent link="${parent}"/>
        <child link="${name}_link"/>
        <axis xyz="0 1 0"/>
        <dynamics damping="0.05" friction="0.05"/>
        <limit upper="1.57" lower="-1.57" effort="1" velocity="10" />
        <origin xyz="${offset_x} ${offset_y} ${offset_z}" rpy="0.0 0.0 0.0"/>
    </joint>
    <link name="${name}_link">
        <inertial>
            <mass value="1e-3" />
            <inertia ixx="1e-5" ixy="0.0" ixz="0.0" iyy="1e-5" iyz="0.0" izz="1e-5"/>
        </inertial>
      <visual>
        <geometry>
          <box size="0.01 0.01 0.01" />
        </geometry>
      </visual>
    </link>
    <gazebo>
      <plugin name="light_gazebo_plugin" filename="libMRSGazeboLightPlugin.so">
        <offset_x>${offset_x}</offset_x>
        <offset_y>${offset_y}</offset_y>
        <offset_z>${offset_z}</offset_z>
        <offset_pitch>${offset_pitch}</offset_pitch>
        <offset_yaw>${offset_yaw}</offset_yaw>
        <offset_roll>${offset_roll}</offset_roll>
        <spawning_frame>${parent}</spawning_frame>
        <update_rate>${update_rate}</update_rate>
        <max_pitch_rate>${max_pitch_rate}</max_pitch_rate>
        <initial_on>${initial_on}</initial_on>
      </plugin>
    </gazebo>
  ...
```

The angle of light can be set by publishing desired angle on topic /uav_name/light/set_pitch e.g.
```
rostopic pub /uav1/light/set_pitch std_msgs/Float32 "data: 1.0"
```
The light can be activated and deactivated by calling service
```
rosservice call /uav1/light/trigger 1/0
```

## Dynamic model plugin

### Description
- Moves a model along predefined trajectory and publishes its current pose

### Usage
After building, activate by adding the following to your model definition.

```xml
  ...
    <plugin name="dynamic_target_plugin" filename="libMRSGazeboDynamicModelPlugin.so">
      <update_rate>30</update_rate>
      <initial_on>true</initial_on>
      <trajectory_file>path_to_trajectory.txt</trajectory_file>
      <loop_enabled>true</loop_enabled>
      <use_segmentation>true</use_segmentation>
      <use_directional_yaw>true</use_directional_yaw>
    </plugin>
  ...
```

The motion of the model can be activated and deactivated by calling service 
```
rosservice call /gazebo/dynamic_model/model_name/activate 1/0
```
reset to initial position of trajectory by calling service 
```
rosservice call /gazebo/dynamic_model/model_name/reset 
```

The trajectory file can be loaded by publishing path to trajectory file on topic /gazebo/dynamic_model/model_name/load_map. 

The trajectory is defined by sequence of tuples "x y z roll pitch yaw velocity", where each tuple is located on a seperate line e.g.
```
  ...
  9.7 2.4 0.0 0.0 0.0 0.0 0.5
  9.6 2.7 0.0 0.0 0.0 1.0 0.5
  9.5 3.0 0.0 0.0 0.0 2.0 0.5
  9.3 3.4 0.0 0.0 0.0 3.0 0.5
  ...
```
