#pragma once

#include "ndt_generic_lamide/eigen_utils.h"

namespace perception_oru
{
namespace graph_localization
{

inline geometry_msgs::PoseArray SigmasToMsg(std::vector<Eigen::Affine3d> sigmas)
{
    geometry_msgs::PoseArray ret;
    for (int i = 0; i < sigmas.size(); i++)
    {
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(sigmas[i], pose);
        ret.poses.push_back(pose);
    }
    ret.header.stamp = ros::Time::now();
    ret.header.frame_id = "world";
    return ret;
}

} // namespace graph_localization
} // namespace perception_oru
