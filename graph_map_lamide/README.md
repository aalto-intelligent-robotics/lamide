# Graph_map
*Part of the Örebro University perception stack*

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons Licence" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.

[![build status](https://gitsvn-nt.oru.se/iliad/software/graph_map/badges/master/build.svg)](https://gitsvn-nt.oru.se/iliad/software/graph_map/commits/master)


## Requires
* NDT tools (https://gitsvn-nt.oru.se/software/ndt_tools)
* NDT core (https://gitsvn-nt.oru.se/software/ndt_core)
* velodyne_pointcloud_oru (https://github.com/dan11003/velodyne_pointcloud_oru)

## Build on ROS-melodic (Ubuntu 18.04LTS)
* Update /usr/include/flann/util/serialization.h to this (remove the flann/ext path):
#include "lz4.h"
#include "lz4hc.h"

This is due to that there is multiple instances of "lz4.h" installed.
