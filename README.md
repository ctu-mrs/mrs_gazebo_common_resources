# MRS resources for Gazebo

> :warning: **Attention please: This README is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

This package contains common gazebo files (worlds, models and plugins) for the [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system/tree/ros2).

## Worlds

- [forest.sdf](worlds/forest.sdf)
- [grass_plane.sdf](worlds/grass_plane.sdf)
- [grass_plane_ardupilot.sdf](worlds/grass_plane_ardupilot) - simulation world with 1ms physics time-step

## Models

- [grass_plane](models/grass_plane)
- [tree_simple](models/tree_simple)

## Plugins

- mrs_multicopter_motor_model - is a Gazebo plugin designed to provide a realistic simulation of multicopter motor behavior
- ardupilot_propeller_plugin - is a Gazebo plugin providing motor simulation and control handle for Ardupilot Gazebo plugin with MRS system
