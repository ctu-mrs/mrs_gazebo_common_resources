# GUI plugins for Gazebo

## The GUI manager plugin

## Description
- Hide the pesky panels in gzclient.
- By modification, add or set GUI elements

### Usage
After building, activate by adding the following to ~/.gazebo/gui.ini
```xml
[overlay_plugins]
filenames=libMRSGazeboGuiManager.so
```
