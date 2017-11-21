# lanelet_rviz_plugin_ros
Rviz Plugin for displaying a [lanelet](https://github.com/phbender/liblanelet) map.

## Installation
* add the package and its dependencies to your workspace
* build it
* source it

## Usage
* Start rviz and add the plugin.
* Lanelet-Map-File (`*.osm`-File) must be specified. Ros parameters are allowed (${myparam}).
* A reference frame and its coordinates (WGS84 latitude/longitude) must be given. Ros parameters are allowed (everything except the first given Parameter in the format ${myparam} will be ignored).
  * The coordinates can also be provided as decimal value.
  * The reference frame does not necessarily need to be the top frame in the tf-tree (nor the fixed frame in RVIZ), but in most cases this is a reasonable choice.
  * It is recommended to set the reference frame globally in the launch file.
* Visibility of the map and sub-elements (e.g. LineStrip-Seperators) can be toggled, without reloading the whole map.

## License
Contact the maintainer.
