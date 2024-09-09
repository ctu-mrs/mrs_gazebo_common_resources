# Sensor and model plugins for Gazebo

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
- A "realistic" mode with more noise and virtually reduced resolution may be activated using the `useRealistic` flag in the `session.yaml` file.

### Usage
The parameter configuration can be found in [mrs_uav_gazebo_simulation](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/tree/master).
If missing, after building, add the following to `generic_components.sdf.jinja`.

```xml
  ...
    <plugin name="{{ camera_name }}{{ camera_suffix }}_plugin" filename="libMrsGazeboCommonResources_RealsensePlugin.so">
      <camera_name>{{ camera_name }}</camera_name>
      <camera_suffix>{{ camera_suffix }}</camera_suffix>

      <useRealistic>{{ realistic }}</useRealistic>
      <minDisparitySGM>1</minDisparitySGM>
      <numDisparitiesSGM>16</numDisparitiesSGM>
      <blockSizeSGM>5</blockSizeSGM>
      <imageScaling>2</imageScaling>
      <depthSaturation>12000</depthSaturation>
      <backgroundNoise>0.1</backgroundNoise>
      <defectKernelSize>5</defectKernelSize>
      <randomNoise>5</randomNoise>

      <parentFrameName>{{ frame_fcu }}</parentFrameName>
      <x>{{ x }}</x>
      <y>{{ y }}</y>
      <z>{{ z }}</z>
      <roll>{{ roll }}</roll>
      <pitch>{{ pitch }}</pitch>
      <yaw>{{ yaw }}</yaw>
    </plugin>
  ...
```

## Servo camera plugin

### Description
- Simulates camera with adjustable pitch and roll angle. The plugin enables to control 2 revolute joints simulating 2-axis gimbal. The plugin requests references to existing links and joints that should be controlled.

### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <gazebo>
      <plugin name="servo_camera_plugin" filename="libMRSGazeboServoCameraPlugin.so">
        <tilt_update_rate>${tilt_update_rate}</tilt_update_rate>
        <max_pitch_rate>${max_pitch_rate}</max_pitch_rate>
        <max_pitch>${max_pitch}</max_pitch>
        <min_pitch>${min_pitch}</min_pitch>
        <max_roll_rate>${max_roll_rate}</max_roll_rate>
        <max_roll>${max_roll}</max_roll>
        <min_roll>${min_roll}</min_roll>
        <joint_name_pitch>${namespace}_servo_camera_joint_pitch</joint_name_pitch>
        <joint_name_roll>${namespace}_servo_camera_joint_roll</joint_name_roll>
        <parent_link_pitch>${namespace}_servo_camera_gimbal_link</parent_link_pitch>
        <parent_link_roll>${parent}</parent_link_roll>
        <compensate_tilt_roll>${compensate_tilt_roll}</compensate_tilt_roll>
        <compensate_tilt_pitch>${compensate_tilt_pitch}</compensate_tilt_pitch>
      </plugin>
    </gazebo>
  ...
```
Complete example of usage including creating links and joints can be found in [MRS robots description file](https://github.com/ctu-mrs/mrs_simulation/blob/master/models/mrs_robots_description/urdf/component_snippets.xacro).

The angle can be set by publishing desired camera angle on topic /uav_name/servo_camera/set_orientation of type std_msgs/Float32MultiArray, where first element of array is required roll angle and second element is required pitch angle, e.g.
```
rostopic pub /uav_name/servo_camera/desired_orientation std_msgs/Float32MultiArray 
"layout:
   dim:
     label: ''
     size: 2
     stride: 0
   data_offset: 0
 data: [0.0, 0.5]"
```

The per axis tilt compensation simulating camera stabilization can be activated and deactivated by calling service /uav_name/servo_camera/compensate_tilt_roll or /uav_name/servo_camera/compensate_tilt_pitch, e.g. 
```
rosservice call /uav1/servo_camera/compensate_tilt_roll "data: true"
```

The camera image is published on topic /uav_name/servo_camera/image_raw.

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
        <compensate_tilt>${compensate_tilt}</compensate_tilt>
      </plugin>
    </gazebo>
  ...
```

The angle of light can be set by publishing desired angle on topic /uav_name/light/set_pitch, e.g.
```
rostopic pub /uav1/light/set_pitch std_msgs/Float32 "data: 1.0"
```
The light can be activated and deactivated by calling service
```
rosservice call /uav1/light/trigger 1/0
```
The tilt compensation simulating perfect light stabilization can be activated and deactivated by calling service /uav_name/light/compensate_tilt, e.g. 
```
rosservice call /uav1/light/compensate_tilt "data: true"
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

## Parachute plugin

### Description
- Adds a deployable emergency parachute to the attached model. WHen packed, it appears as a cylinder.
- Modifies the physics of attached model to significantly increase its drag and dampens the falling velocity.


### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
  <plugin name="parachute_plugin" filename="libMRSGazeboParachutePlugin.so">
    <air_density>1.225</air_density>
    <drag_coeff>500</drag_coeff>
    <cross_section>0.25</cross_section> <!-- [m^2] -->
    <offset_x>0</offset_x>
    <offset_y>0</offset_y>
    <offset_z>-1.56</offset_z>
  </plugin>
  ...
```
Control the parachute by provided trigger services
```
rosservice call /<parent_model_name>/parachute/deploy
```
```
rosservice call /<parent_model_name>/parachute/reset
```

## Water gun plugin

### Description
- Adds a water tank to the model. On activation, the tank will spray spherical particles with a force taken from xml parameters.

### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
  <plugin name="water_gun_plugin" filename="libMRSGazeboWaterGunPlugin.so">
    <muzzle_velocity>${muzzle_velocity}</muzzle_velocity>
    <offset_x>${offset_x}</offset_x>
    <offset_y>${offset_y}</offset_y>
    <offset_z>${offset_z}</offset_z>
    <spread>${spread}</spread>
    <particle_capacity>${particle_capacity}</particle_capacity>
    <spawning_reservoir>${spawning_reservoir}</spawning_reservoir>
  </plugin>
  ...
```
The water gun can be activated by the following service, which will cause it to spray particles infinetly.
```
rosservice call /<parent_model_name>/water_gun/start
```
To stop the spraying, call the stopping service.
```
rosservice call /<parent_model_name>/water_gun/start
```
If you wish to remove all water particles from the scene, use the following service.
```
rosservice call /<parent_model_name>/water_gun/cleanup
```

## Safety LED plugin

Inspired by [`gazebo11/plugins`](https://github.com/osrf/gazebo/tree/gazebo11/plugins): [FlashLightPlugin](https://github.com/osrf/gazebo/blob/gazebo11/plugins/FlashLightPlugin.cc) and [LedPlugin](https://github.com/osrf/gazebo/blob/gazebo11/plugins/LedPlugin.cc).

### Description
- Attaches an RGB LED with a topic-settable color to the model.
- It subscribes to a heartbeat topic `/<parent_model_name>/safety_led/heartbeat` of type [`std_msgs/ColorRGBA`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/ColorRGBA.html) and changes the light color to the message value (transparency is supported).
- Switches back to the default color (red) if heartbeat messages do not arrive within `<failure_duration_threshold>` seconds.

### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <joint name="${name}_joint" type="fixed">
      <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}" />
      <parent link="${parent}" />
      <child link="${name}_link" />
    </joint>
    <link name="${name}_link" >
    </link>
    <gazebo>
      <plugin name="safety_led_plugin" filename="libMRSSafetyLedPlugin.so">
        <model_name>safety_led</model_name>
        <failure_duration_threshold>${failure_duration_threshold}</failure_duration_threshold>
        <model_spawn_delay>${model_spawn_delay}</model_spawn_delay>
        <x>${x}</x>
        <y>${y}</y>
        <z>${z}</z>
        <roll>${roll}</roll>
        <pitch>${pitch}</pitch>
        <yaw>${yaw}</yaw>
      </plugin>
    </gazebo>
  ...
```
