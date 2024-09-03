#pragma once

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>
#include "velodyne_msgs/VelodyneScan.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include "laser_geometry/laser_geometry.h"
#include "queue"
#include "ndt_generic_lamide/utils.h"
#include "pcl/io/pcd_io.h"
#include <velodyne_msgs/VelodyneScan.h>
#include "ndt_generic_lamide/point_types.h"
#include <velodyne_pointcloud_oru/rawdata.h>
#include <velodyne_pointcloud_oru/pointcloudXYZIR.h>


namespace ndt_generic {

template <class PointT>
bool MsgCloud2ToPCL( sensor_msgs::PointCloud2ConstPtr &point_cloud2, pcl::PointCloud<PointT> &cloud);

template <class PointT>
bool MsgCloudToPCL( sensor_msgs::PointCloudConstPtr &point_cloud, pcl::PointCloud<PointT> &cloud);

template <class PointT>
bool LaserScanToPCL( sensor_msgs::LaserScanConstPtr &laser_scan, pcl::PointCloud<PointT> &cloud);


void VelodyneToPcl(velodyne_pointcloud_oru::PointcloudXYZIR &pnts, pcl::PointCloud<pcl::PointXYZL> &cloud);

void VelodyneToPcl(velodyne_pointcloud_oru::PointcloudXYZIR &pnts, pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> &cloud);

template <class PointT> bool UnwarpCloudSimple( velodyne_rawdata_oru::RawData &velodyneparser, velodyne_msgs::VelodyneScan::ConstPtr& scan, pcl::PointCloud<PointT> &cloud, double min_range=0.5, double max_range=130);

void convertPointCloud(const pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> &pnts, pcl::PointCloud<pcl::PointXYZL> &cloud);

void convertPointCloud(const pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> &pnts, pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> &cloud);

pcl::PointXYZL eigenToPCLPoint(const Eigen::Vector3d &pt);

Eigen::Vector3d PCLPointToEigen(const pcl::PointXYZL &pt);

void computeDirectionsAndRangesFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZL> &cloud, const Eigen::Vector3d &origin,
    std::vector<Eigen::Vector3d> &dirs, std::vector<double> &ranges);

template <class PointT>
void filter_fov_fun(pcl::PointCloud<PointT> &cloud,
                    pcl::PointCloud<PointT> &cloud_nofilter, double hori_min,
                    double hori_max);

template <class PointT>
void filter_range_fun(pcl::PointCloud<PointT> &cloud,
                      pcl::PointCloud<PointT> &cloud_nofilter, double range_min,
                      double range_max);

template <class PointT>
void filter_height_angle(pcl::PointCloud<PointT> &cloud,
                         double hori_min=-2*M_PI,
                         double hori_max=2*M_PI, double min_z=DBL_MIN,
                         double max_z=DBL_MAX);

template <typename PointT>
void getMinMax3DPointCloud(const pcl::PointCloud<PointT> &cloud,
                           Eigen::Vector3d &minP, Eigen::Vector3d &maxP);

template <class PointT>
void filter_height_angle(pcl::PointCloud<PointT> &cloud,
                         pcl::PointCloud<PointT> &cloud_nofilter,
                         double hori_min=-2*M_PI,
                         double hori_max=2*M_PI,
                         double min_z=DBL_MIN,
                         double max_z=DBL_MAX);

template <class PointT>
void AddVariance(pcl::PointCloud<PointT> &cloud, double varz=0.05);


template <class PointT>
class PointCloudQueue
{
public:

  PointCloudQueue(size_t size=1) {max_size_=size;}

  void  Push(const pcl::PointCloud<PointT> &cloud);

  void GetCloud(pcl::PointCloud<PointT> &cloud);

  void Save( const std::string &name_prefix="cloud_");

  void Clear(){clouds.clear(); cloud_set_ =0;}

private:
  size_t max_size_;
  std::vector<pcl::PointCloud<PointT> > clouds;
  unsigned int cloud_set_=0;

};

  } // namespace

#include "ndt_generic_lamide/pcl_utils_impl.h"

