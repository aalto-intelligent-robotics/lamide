#pragma once

#include "graph_map_lamide/ndt_dl/point_curv3.h"


namespace perception_oru
{
namespace graph_map
{
const float vertical_lidar_tolerance = 0.1 * M_PI / 18.00;
// typedef velodyne_pointcloud::PointXYZIR PointType;

/*void segmentPointCurvature3(const Eigen::Affine3d &Tsensor, const
pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZL>
> &clouds_out);
/*void segmentPointCurvature3(const Eigen::Affine3d &Tsensor, const pcl::PointCloud<pcl::PointXYZL>
&cloud, std::vector<pcl::PointCloud<pcl::PointXYZL> > &clouds_out);*/

/*void SplitByCurvature(const Eigen::Affine3d &Tsensor,
   std::vector<pcl::PointCloud<velodyne_pointcloud::PointXYZIR> > &ring_clouds,  int &N_SCANS,
                            std::vector<pcl::PointCloud<pcl::PointXYZL> > &clouds_out); */

} // namespace graph_map
} // namespace perception_oru
