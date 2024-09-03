#In this file we define map message that will be published
Header header          #standard header information

NDTMapRGBMsg[] maps     #maps
geometry_msgs/Transform[] transformations #transformation in between each maps
geometry_msgs/Pose[] poses #transformation in between each maps