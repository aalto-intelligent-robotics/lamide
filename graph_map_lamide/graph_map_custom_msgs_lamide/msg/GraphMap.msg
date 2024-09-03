#In this file we define map message that will be published
Header header          #standard header information
geometry_msgs/Pose[] poses
std_msgs/String map_type
ndt_map_lamide/NDTMapRGBMsg[] maps     #cells
