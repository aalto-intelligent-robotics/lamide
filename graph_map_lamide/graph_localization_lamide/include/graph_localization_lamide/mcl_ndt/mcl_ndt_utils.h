#pragma once

#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_localization_lamide/3d_particle_filter.hpp"
#include "time.h"

namespace perception_oru
{
namespace graph_localization
{

inline double getDoubleTime()
{
    struct timeval time;
    gettimeofday(&time, NULL);
    return time.tv_sec + time.tv_usec * 1e-6;
}

inline void normalizeEulerAngles(Eigen::Vector3d& euler)
{
    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);

    if (fabs(euler[0]) > M_PI / 2)
    {
        euler[0] += M_PI;
        euler[1] = -euler[1] + M_PI;
        euler[2] += M_PI;

        euler[0] = angles::normalize_angle(euler[0]);
        euler[1] = angles::normalize_angle(euler[1]);
        euler[2] = angles::normalize_angle(euler[2]);
    }
}

inline void clearNDTCellVec(std::vector<perception_oru::NDTCell*>& vec)
{
    for (size_t i = 0; i < vec.size(); i++)
    {
        if (vec[i] != NULL)
        {
            delete vec[i];
            vec[i] = NULL;
        }
    }
    vec.clear();
}

inline void clearVectorOfNDTCellVec(std::vector<std::vector<perception_oru::NDTCell*>>& vecs)
{
    for (size_t i = 0; i < vecs.size(); i++)
    {
        clearNDTCellVec(vecs[i]);
    }
}

inline void Subsample(std::vector<perception_oru::NDTCell*>& input,
                      std::vector<perception_oru::NDTCell*>& output,
                      double subsample_level_)
{
    if (subsample_level_ != 1)
    {
        for (int i = 0; i < input.size(); ++i)
        {
            double p = ((double)rand()) / RAND_MAX;
            if (p < subsample_level_)
            {
                output.push_back(input[i]);
            }
            else
            {
                delete input[i];
            }
        }
    }
    else
    {
        output = input;
    }
}

inline geometry_msgs::PoseArray ParticlesToMsg(const Eigen::Affine3d& Tmap,
                                               Particles& particles,
                                               const std::string& frame_id = "/world")
{
    geometry_msgs::PoseArray ret;

    ret.poses.resize(particles.size());
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < particles.size(); i++)
        {
            geometry_msgs::Pose particle_msg;
            double x, y, z, r, p, t;
            particles[i].GetXYZ(x, y, z);
            particles[i].GetRPY(r, p, t);
            Eigen::Affine3d pose_particle_wf =
                Tmap * ndt_generic::xyzrpyToAffine3d(x, y, z, r, p, t);
            tf::poseEigenToMsg(pose_particle_wf, particle_msg);
            ret.poses[i] = particle_msg;
        }
        ret.header.stamp = ros::Time::now();
        ret.header.frame_id = frame_id;
    }
    return ret;
}

} // namespace graph_localization
} // namespace perception_oru
