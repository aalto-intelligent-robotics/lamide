#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <angles/angles.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace ndt_generic
{

double L2Distance(const Eigen::Vector3d& mean_a,
                  const Eigen::Matrix3d& cov_a,
                  const Eigen::Vector3d& mean_b,
                  const Eigen::Matrix3d& cov_b);

}