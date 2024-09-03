#pragma once

#include <ndt_map_lamide/cell_vector.h>
#include <ndt_map_lamide/lazy_grid.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/pointcloud_utils.h>
//#include <ndt_registration_lamide/ndt_matcher_d2d.h>
//#include <ndt_registration_lamide/ndt_matcher_d2d_feature.h>

namespace ndt_rviz {

using perception_oru::NDTCell;
using perception_oru::NDTMap;
using perception_oru::CellVector;
using perception_oru::LazyGrid;

double getRobustYawFromAffine3d(const Eigen::Affine3d &a);

void distanceBetweenAffine3d(const Eigen::Affine3d &p1,
                             const Eigen::Affine3d &p2, double &dist,
                             double &angularDist);

Eigen::Affine2d eigenAffine3dTo2d(const Eigen::Affine3d &a3d);

Eigen::Affine3d eigenAffine2dTo3d(const Eigen::Affine2d &a2d);

Eigen::Affine3d forceEigenAffine3dTo2d(const Eigen::Affine3d &a3d);

void forceEigenAffine3dTo2dInPlace(Eigen::Affine3d &a3d);

Eigen::Affine2d getAffine2d(double x, double y, double th);

Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>
ICPwithCorrMatch(perception_oru::NDTMap &targetNDT,
                 perception_oru::NDTMap &sourceNDT,
                 const std::vector<std::pair<int, int> > &corresp);

void printTransf(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

void printTransf2d(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

void convertAffineToVector(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T,
    Eigen::Matrix<double, 6, 1> &vec);

void addNDTCellToMap(NDTMap *map, NDTCell *cell);

//! Note: this will overwrite any existing cells
void setNDTCellToMap(NDTMap *map, NDTCell *cell);

Eigen::Vector3d computeLocalCentroid(const Eigen::Vector3d &map_centroid,
                                     const Eigen::Vector3d &local_pos,
                                     double resolution);

bool discardCell(NDTMap &map, const pcl::PointXYZL &pt);

std::string transformToEvalString(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

std::string transformToEval2dString(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

} // namespace
