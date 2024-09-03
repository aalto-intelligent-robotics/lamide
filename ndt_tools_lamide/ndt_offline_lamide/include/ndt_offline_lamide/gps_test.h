#pragma once
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <cmath>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/Imu.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "eigen_conversions/eigen_msg.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include "stdio.h"
#include <ublox_msgs/NavPOSECEF.h>
#define foreach BOOST_FOREACH

#include <iostream>
#include <exception>
#include <cmath>
bool read_bag(const std::string &bag_path, std::vector<sensor_msgs::NavSatFix> &buffer);
