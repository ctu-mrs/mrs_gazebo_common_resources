# Model plugins for Gazebo

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
