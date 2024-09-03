# Using graph_map to create an NDT-map

This tutorial will show how to use the Graph_map package to create maps for accurate robot localization using the method [Normal Distributions Transform Occupancy Map Fusion](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.412.328&rep=rep1&type=pdf). Graph_map is a reimplementation of NDT-Fuser and support 2d/3d mapping using laser range measurments in ROS.


## Getting Started


### Prerequisites

The package requires [ros kinetic](http://wiki.ros.org/kinetic/Installation) and has been tested on ubuntu 16.04.
The tutorial optionally require wget.
```
sudo apt-get install wget
```
### Installing the packages

Open a terminal and move into the source folder of your catkin workspace, clone the perception_oru repository.
```
git clone https://gitsvn-nt.oru.se/software/perception_oru
cd ..
catkin_make
```
#### Downloading tutorial data

Data for 2d or 3d mapping can be found at our lab website https://mro.oru.se/software/ . Download the file [2018-05-18-09-55-16.bag](https://mro.oru.se/wp-content/material/data/2018-05-18-09-55-16.bag) using e.g. your browser or wget. The file should be placed under graph_map/data .

```
roscd graph_map/data
wget https://mro.oru.se/wp-content/material/data/2018-05-18-09-55-16.bag
```

## Running the tests

### Mapping with 2d data

```
roslaunch graph_map 2d_fuser_oru_example.launch run_bag:=true
```
Starts up the graph_map node, they optional argument run_bag:=true will replay the bag file.

### Saving a map

```
rosservice call /graph_node/save_map
```
The map will be stored under graph_map/maps.
## Authors
This guide was written by Daniel Adolfsson from Örebro Univiresity based on the [previous work](http://wiki.ros.org/perception_oru/Tutorials/Using%20NDT%20Fuser%20to%20create%20an%20NDT%20map) by Todor Stoyanov et al. Graph_map is currently maintained by:

* **[Daniel Adolfsson](https://www.linkedin.com/in/daniel-adolfsson-7613417a/)** - *(daniel.adolfsson@oru.se)* - Örebro University, Sweden
* **Henrik Andreasson** - Örebro University, Sweden

The NDT-Core packages are developed and supported by the Perception_oru group from [MR&O Lab](http://www.aass.oru.se/Research/mro/index.html).
See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

### Contributors
### References

