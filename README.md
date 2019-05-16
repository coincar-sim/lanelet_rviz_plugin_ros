[![Build Status](https://api.travis-ci.org/coincar-sim/lanelet_rviz_plugin_ros.svg)](https://travis-ci.org/coincar-sim/lanelet_rviz_plugin_ros)

# lanelet_rviz_plugin_ros
Rviz Plugin for displaying a [lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) map.

## Installation
* add the package and its dependencies to your workspace
* build it
* source it

## Usage
* Start rviz and add the plugin.
* The lanelet2 map file must be specified via `lanelet2_interface_ros`.
* Visibility of the map and sub-elements (e.g. LineStrip-Seperators) can be toggled, without reloading the whole map.
* Launch `$ roslaunch lanelet_rviz_plugin_ros sample_lanelet_viz.launch` for an example.
* *Note:* For large lanelet2 maps, loading the map takes some seconds, which slows down the rviz initialization.

## Contributors
Pascal Böhmler, Alexander Naumann, Maximilian Naumann

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
