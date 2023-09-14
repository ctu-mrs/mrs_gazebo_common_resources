# Physics plugins for Gazebo

This directory includes plugins that somehow modify the physics behavior of the UAV or its parts.

## Plugins
* fluid_resistace_plugin
* motor_prop_model_plugin

### Fluid Resistance
This plugin models linear resistance of the air (Fd = model_mass * resistance_constant * linear_body_velocity).
You can set resistance coefficientsa (for every axis in body frame) in model definition using fluid resistance macro.
It is also possible to change fluid resistance while simulation is running using topic */fluid_resistance*.
```
rostopic pub /fluid_resistance geometry_msgs/Vector3 "x: 0.0 
y: 0.0
z: 0.0" 
```

### Motor Propeller Model 
This plugin models the motor with propeller. 
Currently it is just modificaiton of the Fadri Furre's work, which was slightly modified for our needs.

*Changes*

- Motor speed republisher was added.
- Linear limitation of the motor's generated force was removed (it was modeled such as the motor's force was zero at 25 m/s).

