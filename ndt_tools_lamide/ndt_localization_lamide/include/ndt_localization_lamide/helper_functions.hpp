#ifndef HELPER_FUNCTIONS_HPP_
#define HELPER_FUNCTIONS_HPP_
#include <cmath>
#include <Eigen/Dense>

template <typename T> T sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

void to2PI(double &a);

void toPI(double &a);

void EigenSort( Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors );

//This function sorts the egien vectors that first will be tangent, then normal and the last is perpendicular to XY plane
void EigenSort2D( Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors );

Eigen::Affine3d getAsAffine(float x, float y, float yaw );

#endif
