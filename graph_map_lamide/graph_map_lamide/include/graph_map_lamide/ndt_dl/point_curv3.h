#pragma once

#include "ndt_generic_lamide/point_types.h"
#include "velodyne_pointcloud_oru/pointcloudXYZIR.h"

#include <angles/angles.h>
#include <ndt_generic_lamide/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>



namespace perception_oru
{
namespace graph_map
{

typedef velodyne_pointcloud_oru::PointXYZIR PointType;

const float vertical_lidar_tolerance = 0.1 * M_PI / 18.00;

// input velodyne, output velodyne
void segmentPointCurvature3(const Eigen::Affine3d& Tsensor,
                            const pcl::PointCloud<PointType>& cloud,
                            std::vector<pcl::PointCloud<PointType>>& clouds_out);

// input velodyne, output pcl
void segmentPointCurvature3(const Eigen::Affine3d& Tsensor,
                            const pcl::PointCloud<PointType>& cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds_out);
// input pcl, output pcl
void segmentPointCurvature3(const Eigen::Affine3d& Tsensor,
                            const pcl::PointCloud<pcl::PointXYZL>& cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds_out);

void SegmentRings(const pcl::PointCloud<pcl::PointXYZL>& cloud,
                  std::vector<pcl::PointCloud<PointType>>& ring_clouds,
                  int& N_SCANS);

void SegmentRings(const pcl::PointCloud<PointType>& cloud,
                  std::vector<pcl::PointCloud<PointType>>& ring_clouds,
                  int& N_SCANS);

void ClassifyPoints(const Eigen::Affine3d& Tsensor,
                    std::vector<pcl::PointCloud<PointType>>& ring_clouds,
                    const int& N_SCANS,
                    std::vector<pcl::PointCloud<PointType>>& clouds_out);

bool similar(const float& x, const float& y);

void ToXYZ(const std::vector<pcl::PointCloud<PointType>>& cloud,
           std::vector<pcl::PointCloud<pcl::PointXYZL>>& cloud_out);

} // namespace graph_map
} // namespace perception_oru
