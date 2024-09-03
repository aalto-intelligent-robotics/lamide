#pragma once

#include <eigen_conversions/eigen_msg.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ndt_rviz_lamide/utils.h>

namespace ndt_rviz {

// Contains a set of useful functions to generate markers from NDT related
// classes.

// Some helper functions to convert to the position etc. utilized in the
// markers.
geometry_msgs::Point toPointFromPCL(const pcl::PointXYZL &p);

geometry_msgs::Point toPointFromTF(const tf::Vector3 &p);

geometry_msgs::Point toPointFromEigen(const Eigen::Vector3d &p);

geometry_msgs::Point toPointFromEigen(const Eigen::Affine3d &T);

void assignDefault(visualization_msgs::Marker &m);

void assignColor(visualization_msgs::Marker &m, int color);

// Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
visualization_msgs::Marker markerNDTCells(
    std::vector<perception_oru::NDTCell *> cells /*, tf::Pose& pose*/,
    const visualization_msgs::Marker &marker);

// Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
inline visualization_msgs::Marker
markerNDTCells(std::vector<perception_oru::NDTCell *> cells,
               const Eigen::Affine3d &pose,
               const visualization_msgs::Marker &marker);

inline visualization_msgs::Marker
markerNDTCells(std::vector<perception_oru::NDTCell *> cells);

inline visualization_msgs::Marker
markerNDTCells(perception_oru::NDTMap &map, int id, const std::string &name);

inline visualization_msgs::Marker markerNDTCells(perception_oru::NDTMap &map,
                                                 const Eigen::Affine3d &pose,
                                                 int id,
                                                 const std::string &name);

inline visualization_msgs::Marker markerNDTCells(perception_oru::NDTMap &map,
                                                 int id);

// Visualize the ndt cells as a set of 3 lines drawn along the eigen vectors.
void markerNDTCells2(std::vector<perception_oru::NDTCell *> cells,
                     const Eigen::Affine3d &pose,
                     visualization_msgs::Marker &m);

void markerNDTCells2(perception_oru::NDTMap &map, const Eigen::Affine3d &pose,
                     int id, const std::string &name,
                     visualization_msgs::Marker &m);

//! Draw correspondance lines between the NDTCells
inline visualization_msgs::Marker
markerCellVectorCorrespondances(perception_oru::NDTMap &map1,
                                perception_oru::NDTMap &map2,
                                const std::vector<std::pair<int, int> > &corr);

// Code from user skohlbrecher
void drawCovariance(const Eigen::Vector2d &mean,
                    const Eigen::Matrix2d &covMatrix,
                    visualization_msgs::Marker &marker);

void drawCovariance(const Eigen::Vector3d &mean,
                    const Eigen::Matrix3d &covMatrix,
                    visualization_msgs::Marker &marker);

inline visualization_msgs::Marker
markerMeanCovariance2d(const Eigen::Vector3d &mean, const Eigen::Matrix3d &cov,
                       double scale, int id, int color);

// Visualization markers.
void appendMarkerArray(visualization_msgs::MarkerArray &array,
                       const visualization_msgs::MarkerArray &add);

visualization_msgs::Marker getMarkerArrowAffine3d(const Eigen::Affine3d &T,
                                                  int id, int color,
                                                  const std::string &ns);

visualization_msgs::Marker getMarkerCylinder(const Eigen::Affine3d &T, int id,
                                             int color, double length,
                                             double radius,
                                             const std::string &ns);

/// Draw an x,y,z coordsystem given an affine3d.
visualization_msgs::MarkerArray getMarkerFrameAffine3d(const Eigen::Affine3d &T,
                                                       const std::string &ns,
                                                       double length,
                                                       double radius);

visualization_msgs::Marker
getMarkerLineListFromTwoPointClouds(const pcl::PointCloud<pcl::PointXYZL> &pc1,
                                    const pcl::PointCloud<pcl::PointXYZL> &pc2,
                                    int color, const std::string &ns,
                                    const std::string &frame_id, double width);

} // namespace
