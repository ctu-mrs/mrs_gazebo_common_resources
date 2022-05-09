^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_gazebo_common_resources
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2022-05-09)
------------------
* fixes for lidars
* + install in cmakelists
* removed person model, too large
* adding mrs_city world
  Updated plugin: Servo camera
* Fix roll limits and update readme
* Change topic for control of servo camera
* Add 2-axis tilt compensation
* Update servo camera plugin to 2-axis control
* SafetyLedPlugin: melodic backward compability
* Update servo camera plugin
  New plugin: safety/diagnostics LED
* add models/safety_led
* update READMEs with SafetyLedPlugin
* safety_led_plugin: add light/visual color modificators
* SafetyLedPlugin: initial files; untested
* updated realsense plugin naming and loading
* disabled clouds in the forest world, they brake 3d sensors
* updated naming of the realsense plugin
* [3dlidar plugin]: minor refactoring
  added dependency on own built protobuf msg into magnetometer plugin
* added dependency on own built protobuf msg into magnetometer plugin
* + 'reflectivity' & 'ambient' plchdlrs to 3dlidr PC
* added custom mag plugin
* Contributors: Dan Hert, Matouš Vrba, Pavel Petracek, Pavel Petráček, Tomas Baca, Vit Kratky, Vojtech Spurny, Vít Krátký

1.0.1 (2021-05-16)
------------------
* add camera calibration grid pattern (matches physically the one in lab)
* added realsense_prefix parameter to realsense_plugin to be able to differentiate between upper and lower realsense
* Contributors: Pavel Petracek, Tomas Baca, Vaclav Pritzl

1.0.0 (2021-03-18)
------------------
* Major release
