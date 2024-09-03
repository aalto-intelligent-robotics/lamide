# Using NDT-MCL for localisation in NDT maps.

This tutorial will show how to use the Graph_localization package to for robust localisation in NDT maps.

### Prerequisites
This tutorial requires that you've completed teh Graph_map tutorial to create a map.

## Running the localisation node
Launch the following command to start the NDT-MCL node, the optional argument run_bag:=true will replay the bag file specified in the launch file.
```
roslaunch graph_localization 2d_mcl_oru_lab.launch run_bag:=true
```
The NDT Map will be loaded from the path "Graph_map/maps/ndt_map.MAP".
The estimate is initialised using the ground truth pose estimate from the topic "/robot1/kmo_navserver/state". The pose estimate can later be reinitialised using the "2D Pose Estimate" button in the in Rviz frontpanel.

## Authors
Graph_localization is currently maintained by:

* **[Daniel Adolfsson](https://www.linkedin.com/in/daniel-adolfsson-7613417a/)** - *(daniel.adolfsson@oru.se)* - Örebro University, Sweden
* **Henrik Andreasson** - Örebro University, Sweden

The NDT-Core packages are developed and supported by the Perception_oru group from [MR&O Lab](http://www.aass.oru.se/Research/mro/index.html).
See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

### Contributors
### References

