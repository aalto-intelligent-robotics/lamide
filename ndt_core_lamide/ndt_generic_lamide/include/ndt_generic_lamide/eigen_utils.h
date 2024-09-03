#pragma once

#include "cfloat"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <angles/angles.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace ndt_generic {

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >
    Affine3dSTLVek;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

double GetDistance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);

double getCondition(const Eigen::MatrixXd &m);

void normalizeEulerAngles(Eigen::Vector3d &euler);

double GetDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

void normalizeEulerAngles6dVec(Eigen::VectorXd &v);

double SearchForClosestElement(const Eigen::Affine3d &pose,
                               const Affine3dSTLVek &affine3d_vek,
                               Eigen::Affine3d &closest_pose, bool &node_found);

double SearchForClosestElement(const Eigen::Affine3d &pose,
                               const Affine3dSTLVek &affine3d_vek);
std::string transformToEvalString(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);
std::string transformToEvalStringNoLineEnd(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

std::string affine3dToStringRPY(const Eigen::Affine3d &T);

std::string affine3dToStringRotMat(const Eigen::Affine3d &T);

void typeCastStdVectorToEigen(std::vector<double> &v, Eigen::VectorXd &ev);

Eigen::Affine3d getAffine3dMean(const std::vector<Eigen::Affine3d> &Ts);

// In case the weights have large variance this is likely to fail - use
// getAffine3dMeanWeightsUsingQuat() instead.
Eigen::Affine3d getAffine3dMeanWeights(const std::vector<Eigen::Affine3d> &Ts,
                                       const std::vector<double> &weights);

// ORU_BUG - check this, has to be a bug here...
Eigen::Affine3d
getAffine3dMeanWeightsUsingQuat(const std::vector<Eigen::Affine3d> &Ts,
                                const std::vector<double> &weights);

Eigen::Affine3d
getAffine3dMeanWeightsUsingQuatNaive(const std::vector<Eigen::Affine3d> &Ts,
                                     const std::vector<double> &weights);

Eigen::Affine3d vectorToAffine3d(const Eigen::VectorXd &v);

Eigen::Affine3d vectorsToAffine3d(const Eigen::Vector3d &transl,
                                  const Eigen::Vector3d &rot);


Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v);

Eigen::Affine3d xyzrpyToAffine3d(double x, double y, double z, double r, double p, double t);

Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond &q);

Eigen::VectorXd affine3dToVector(const Eigen::Affine3d &T);

void affine3dToVectors(const Eigen::Affine3d &T, Eigen::Vector3d &transl,
                       Eigen::Vector3d &rot);

Eigen::Vector3d getWeightedPoint(const Eigen::Vector3d &pt1,
                                 const Eigen::Matrix3d &cov1,
                                 const Eigen::Vector3d &pt2,
                                 const Eigen::MatrixXd &cov2);

Eigen::Affine3d getWeightedPose(const Eigen::Affine3d &pose1,
                                const Eigen::MatrixXd &cov1,
                                const Eigen::Affine3d &pose2,
                                const Eigen::MatrixXd &cov2);

double getYaw(const Eigen::Affine3d &T);

void updateAffineRotationFromEuler(Eigen::Affine3d &T, Eigen::Vector3d &euler);

void updateRollPitch(Eigen::Affine3d &T, Eigen::Vector3d &euler);

Eigen::Affine3d MuxAffineComponents(const Eigen::Affine3d &T1, const Eigen::Affine3d &T2, bool t1x, bool t1y, bool t1z, bool t1ex, bool t1ey, bool t1ez);

/*!
 * \brief Extrapolate
 * \param T1
 * \param T2
 * \param stepsize
 * \return
 */
Eigen::Affine3d Extrapolate(const Eigen::Affine3d &T1,
                            const Eigen::Affine3d &T2, double stepsize = 1.0);
Eigen::Affine3d Interpolate(const Eigen::Affine3d &T1,
                            const Eigen::Affine3d &T2, double factor = 0.5);
Eigen::Affine3f Interpolate(const Eigen::Affine3f &T1,
                            const Eigen::Affine3f &T2, double factor = 0.5);

bool AlmostEqual(double a, double b);

Eigen::Matrix3d Cov6dTo3d(const Eigen::Matrix<double,6,6> &Cov);

} // namespace
