#ifndef READPOINTCLOUD_H
#define READPOINTCLOUD_H
#include "string.h"
#include "stdio.h"
#include <ros/ros.h>
//#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud_oru/rawdata.h>
#include <ndt_generic_lamide/point_types.h>
#include <iostream>
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>
#include <rosbag/view.h>
#include "ndt_offline_lamide/imu_prediction.h"
#include "pcl/point_cloud.h"
#include <pcl/common/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include "ndt_generic_lamide/eigen_utils.h"
#include "nav_msgs/Odometry.h"
#include "ndt_generic_lamide/pcl_utils.h"
#include <ndt_generic_lamide/point_types.h>
#include <velodyne_pointcloud_oru/pointcloudXYZIR.h>
#include <velodyne_pointcloud_oru/rawdata.h>


#include "laser_geometry/laser_geometry.h"
#include <ndt_offline_lamide/PoseInterpolationNavMsgsOdo.h>


using std::string;
namespace ndt_offline{
typedef enum odometrytype {NO_ODOM=0, IMU=1,WHEEL_ODOM=2}OdometryType;
class readPointCloud
{
public:


  //readPointCloud(const std::string &bag_path);

  readPointCloud(const std::string &bag_path, const Eigen::Affine3d &T_sensor, const OdometryType odometry_sensor, const std::string &lidar_topic, double min_range, double max_range, const std::string &velodyne_config, double t_offset, const std::string &tf_topic="tf", const std::string &fixed_frame_id="world", const std::string &interpolation_link_id="", bool unpack_raw = false);

  ros::Time GetTimeOfLastCloud();

  template<class PointT> bool readNextMeasurement(pcl::PointCloud<PointT> &cloud);

  bool GetOdomPose(const ros::Time &t, const std::string &frame_id, Eigen::Affine3d &pose);

  bool GetOdomPose(const ros::Time &t, const std::string &frame_id, const std::string &fixed_frame_id, Eigen::Affine3d &pose);

  bool SkipToCloudIndex(const unsigned int index=0);

private:

  template<class PointT> void UnwarpCloudWheelOdom(pcl::PointCloud<PointT> &cloud);

  template<class PointT> bool UnwarpVelodyneMsgOdomUndistortion(velodyne_msgs::VelodyneScan::ConstPtr &scan,pcl::PointCloud<PointT> &cloud);

  template<class PointT> bool UnwarpVelodyneMsgImuUndistortion(velodyne_msgs::VelodyneScan::ConstPtr &scan,pcl::PointCloud<PointT> &cloud);
/*
  template<class PointT> void OutputPoint(PointT &point);

  template<>
  void OutputPoint(velodyne_pointcloud_oru::PointXYZIR &point);
*/






  //Odometry sensing
  string imu_topic_,lidar_topic_;
  string interpolation_link_id_; //for undistortion using odomety
  OdometryType  odometry_sensor_;
  ndt_offline::imu_prediction *imu_pred_;
  PoseInterpolationNavMsgsOdo *odom_lookup;

  //Laser
  velodyne_rawdata_oru::RawData *velodyne_parser_;
  laser_geometry::LaserProjection projector_;

  //Rosbag related
  rosbag::Bag bag_;
  rosbag::View *view_;
  rosbag::View::iterator I;

  ros::Time t_cloud;
  double min_range_,max_range_, t_offset_;

  ros::NodeHandle nh;
  Eigen::Affine3d Tsensor_;







};
}
#include "ndt_offline_lamide/readpointcloud_impl.h"
#endif // READPOINTCLOUD_H
