# lanelet_rviz_plugin_ros
Rviz Plugin for displaying a [lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) map.

## Installation
* add the package and its dependencies to your workspace
* build it
* source it

## Usage
* Start rviz and add the plugin.
* Lanelet-Map-File (`*.osm`-File) must be specified. Ros parameters are allowed (${myparam}).
* A reference frame and its origin geo coordinates (latitude/longitude) must be given via a NavSatFix-Message
* Visibility of the map and sub-elements (e.g. LineStrip-Seperators) can be toggled, without reloading the whole map.
* launch `$ roslaunch lanelet_rviz_plugin_ros sample_lanelet_viz.launch` for an example

## Contributors
Pascal Böhmler, Alexander Naumann, Maximilian Naumann

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
