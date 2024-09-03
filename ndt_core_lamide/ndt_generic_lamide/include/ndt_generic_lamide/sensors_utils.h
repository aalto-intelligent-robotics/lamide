#pragma once
#include "Eigen/Dense"
#include "stdio.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_datatypes.h"

namespace ndt_generic {

//bool GetSensorPose(Eigen::Affine3d &sensor, const std::string &dataset="", Eigen::Vector3d &transl=Eigen::vector3d(0,0,0),  Eigen::Vector3d &euler=Eigen::vector3d(0,0,0));
bool GetSensorPose(const std::string &dataset,  Eigen::Vector3d & transl,  Eigen::Vector3d &euler,tf::Transform &Tsensor_tf, Eigen::Affine3d &Tsensor_eig);

}
