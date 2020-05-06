# World plugins for Gazebo

## Rviz camera synchronizer plugin

### Description
- Synchronize rviz camera position and orientation according to gazebo camera

### Usage
After building, activate by adding the following to your `world` definition.
```xml
  ...
  <world name="...">
  ...
    <plugin name="mrs_gazebo_rviz_camera_synchronizer" filename="libMRSGazeboRvizCameraSynchronizer.so" >
      <target_frame_id>gazebo_user_camera</target_frame_id>
      <world_origin_frame_id>uav1/gps_origin</world_origin_frame_id>
      <frame_to_follow>uav1::base_link</frame_to_follow>
    </plugin>
  ...
```

## Static transform republisher plugin

### Description
- Republish tf messages from sensors from topic `\tf_gazebo_static` to `\tf_static` using [tf2::StaticTransformBroadcaster](http://docs.ros.org/diamondback/api/tf2_ros/html/classtf2_1_1StaticTransformBroadcaster.html "Class definition")

### Usage
After building, activate by adding the following to your `world` definition.
```xml
  ...
  <world name="...">
  ...
    <plugin name="mrs_gazebo_static_transform_republisher_plugin" filename="libMRSGazeboStaticTransformRepublisher.so"/>
  ...
```
