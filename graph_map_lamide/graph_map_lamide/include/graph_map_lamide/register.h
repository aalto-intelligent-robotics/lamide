#pragma once
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/ndt/ndtd2d_reg_type.h"
#include "graph_map_lamide/reg_type.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "stdio.h"

#include <pcl/point_cloud.h>
//#include "gnuplot-iostream.h"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/shared_ptr.hpp"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "ndt_generic_lamide/motion_model_2d.h"
#include "ndt_generic_lamide/motion_model_3d.h"
#include "ndt_map_lamide/ndt_map.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
//#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/pcl_utils.h"
#include "pcl/common/transforms.h"
#include "ros/node_handle.h"
#include "ros/time.h"


namespace perception_oru
{
namespace graph_map
{

class Registration
{
public:
    Registration(const std::string& maptype, const std::string& registratorType);

    template <class PointT>
    bool scanmatching(std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<PointT>>>& reference,
                      pcl::PointCloud<PointT>& source,
                      Eigen::Affine3d& Tsrc,
                      Eigen::MatrixXd& reg_cov);

    template <class PointT>
    bool scanmatching(pcl::PointCloud<PointT>& target,
                      pcl::PointCloud<PointT>& source,
                      Eigen::Affine3d& Tsrc,
                      Eigen::MatrixXd& reg_cov,
                      const Eigen::Affine3d& Ttaraget = Eigen::Affine3d::Identity());

    template <class PointT>
    bool exaustiveMatching(pcl::PointCloud<PointT>& target,
                           pcl::PointCloud<PointT>& source,
                           Eigen::Affine3d& Tsrc,
                           Eigen::MatrixXd& reg_cov,
                           const Eigen::Affine3d& Ttaraget = Eigen::Affine3d::Identity());

    std::string ToString()
    {
        return registrator_->ToString();
    }

protected:
    template <class PointT>
    void Aggregate(std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<PointT>>>& reference,
                   pcl::PointCloud<PointT>& aggregated_ref);

    template <class PointT>
    void ConvertAndTransform(pcl::PointCloud<PointT>& cld_in,
                             pcl::PointCloud<pcl::PointXYZL>& cld_out,
                             Eigen::Affine3d& Toffset);

    bool KeyFrameBasedFuse(const Eigen::Affine3d& Tdiff); // only fuse if

    std::string maptype_, registratorType_;
    RegParamPtr regParam_;
    RegTypePtr registrator_;
    ros::NodeHandle nh_;
    bool use_keyframe_;
    double score = 0;
    double min_keyframe_dist_, min_keyframe_rot_deg_;
};

} // namespace graph_map

} // namespace perception_oru
#include "graph_map_lamide/register_impl.h"
