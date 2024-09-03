#pragma once
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/reg_type.h"
#include "ndt_generic_lamide/labels.h"
#include "graphfactory.h"
#include "iostream"
#include "ndt/ndtd2d_reg_type.h"
#include "ndt/ndtd2d_reg_type_lamide.h"
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
#include "ros/time.h"

namespace perception_oru
{
namespace graph_map
{

using std::cerr;
using std::cout;
using std::endl;

using Eigen::Affine3d;
using perception_oru::MotionModel2d;
using perception_oru::MotionModel3d;
class GraphMapFuser
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphMapFuser()
    {
    }

    GraphMapFuser(string maptype,
                  string registratorType,
                  const Eigen::Affine3d& init_pose,
                  const Affine3d& sensorPose,
                  const std::string& path,
                  bool ndt_om = true,
                  bool only_static = false); // Ros friendly constructor to read parameters from
                                       // ros parameter server

    GraphMapFuser(RegParamPtr regParam,
                  MapParamPtr mapParam,
                  GraphMapNavigatorParamPtr graph_param,
                  const Eigen::Affine3d& init_pose,
                  const Eigen::Affine3d& sensorPose,
                  const std::string& path,
                  bool ndt_om = true,
                  bool only_static = false);

    Affine3d GetPoseLastFuse() const
    {
        return pose_last_fuse_;
    }

    void SaveGraphMap(const std::string& path, const std::string& filename, const std::string& prefix = "");

    void SavePointCloud(const std::string& path, const std::string& filename);

    void SaveCurrentNodeAsJFF(const std::string& filename);

    void SetMotionModel(const MotionModel3d& motion_model)
    {
        motion_model_3d_ = motion_model;
    }

    void SetFuserPose(const Eigen::Affine3d& pose)
    {
        pose_last_fuse_ = pose;
    }

    const GraphMapNavigatorPtr GetGraph() const
    {
        return graph_map_;
    }

    void PlotMapType(MapTypePtr map);

    Eigen::MatrixXd PredictOdomUncertainty(const Eigen::Affine3d Tnow, bool prediction2d = true);

    //!
    //! //! \brief ProcessFrame
    //! //! \param cloud
    //! //! \param Tnow
    //! //! \param Tmotion robot movement
    //! //! \return true if registration was succesfull and map was updated
    //! bool ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow, Eigen::Affine3d
    //! &Tmotion); //cloud is the current scan in robot frame,  Tnow is the current pose in world
    //! frame

    template <class PointT>
    bool ProcessFrame(pcl::PointCloud<PointT>& cloud,
                      Eigen::Affine3d& Tnow,
                      Eigen::Affine3d& Tmotion);

    template <class PointT>
    bool ProcessFrame(pcl::PointCloud<PointT>& cloud,
                      Eigen::Affine3d& Tnow,
                      Eigen::Affine3d& Tmotion,
                      Eigen::MatrixXd& Tcov);

    template <class PointT>
    bool ProcessFrameStatic(pcl::PointCloud<PointT>& cloud,
                            Eigen::Affine3d& Tnow,
                            Eigen::Affine3d& Tmotion);

    template <class PointT>
    bool ProcessFrameStaticGlobalMap(pcl::PointCloud<PointT>& cloud,
                                     Eigen::Affine3d& Tnow,
                                     Eigen::Affine3d& Tmotion);

    template <class PointT>
    void UpdateMultipleMaps(pcl::PointCloud<PointT>& cloud, Eigen::Affine3d& Tnow);

    template <class PointT>
    void UpdateSingleMap(pcl::PointCloud<PointT>& cloud, Eigen::Affine3d& Tnow);

    // template<class PointT>  bool scanmatching(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d
    // &Tnow, Eigen::Affine3d &Tmotion);

    //----
    template <class PointT>
    bool ProcessFrame(std::vector<pcl::PointCloud<PointT>>& clouds,
                      Eigen::Affine3d& Tnow,
                      Eigen::Affine3d& Tmotion);

    template <class PointT>
    bool ProcessFrame(std::vector<pcl::PointCloud<PointT>>& clouds,
                      Eigen::Affine3d& Tnow,
                      Eigen::Affine3d& Tmotion,
                      Eigen::MatrixXd& Tcov);

    template <class PointT>
    bool ProcessFrameStatic(std::vector<pcl::PointCloud<PointT>>& clouds,
                            Eigen::Affine3d& Tnow,
                            Eigen::Affine3d& Tmotion);

    template <class PointT>
    bool ProcessFrameStaticGlobalMap(std::vector<pcl::PointCloud<PointT>>& clouds,
                                     Eigen::Affine3d& Tnow,
                                     Eigen::Affine3d& Tmotion);

    template <class PointT>
    void UpdateMultipleMaps(std::vector<pcl::PointCloud<PointT>>& clouds, Eigen::Affine3d& Tnow);

    template <class PointT>
    void UpdateSingleMap(std::vector<pcl::PointCloud<PointT>>& clouds, Eigen::Affine3d& Tnow);

    /*!
     * \brief scanmatching Can be used rather than ProcessFrame for simple scan matching between a
     * reference and a set of source scans \param reference \param source \param Tmotion \return
     */
    bool scanmatching(
        std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<pcl::PointXYZL>>>& reference,
        pcl::PointCloud<pcl::PointXYZL>& source,
        Eigen::Affine3d& Tnow,
        Eigen::Affine3d& Tcorrected,
        Eigen::MatrixXd& reg_cov,
        size_t submap_size = 3);

    MapTypePtr CreateMap(
        std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<pcl::PointXYZL>>>& reference);

    unsigned int FramesProcessed() const
    {
        return nr_frames_;
    }

    void AddPoseFramePair(const Eigen::Affine3d& Tnow, const Eigen::Affine3d& Tref);

    void PlotMapType();

    bool ErrorStatus(string status = "");

    bool FuseFrame(const Affine3d& Tnow, const Affine3d& Tmotion);

    bool KeyFrameBasedFuse(const Affine3d& Tnow);

    bool FuseNoMotionFrames(const Affine3d& Tnow, const Affine3d& Tmotion);

    void Visualize(bool enableVisualOutput, plotmarker marker = plotmarker::sphere);

    void SetFuserOptions(bool save_merged_cloud = false);

    std::string ToString();

    void SetkeyframeOptions(bool static_scans_only)
    {
        fuse_no_motion_frames = static_scans_only;
    }

    void plotErrorMaps(const pcl::PointCloud<pcl::PointXYZL>& cloud_localized,
                       const Eigen::Affine3d& pose);

    void checkLabelConsistency(const pcl::PointCloud<pcl::PointXYZL>& cloud_localized,
                               const Eigen::Affine3d& pose);

    void ColorVoxelsWithScore(std::vector<perception_oru::NDTCell*>& ndts,
                              const Eigen::Affine3d& pose);

    void ColorVoxelsWithLabel(std::vector<perception_oru::NDTCell*>& ndts);

    void setUseOdomAsGT(bool use);

    void setClusterUpdate(bool cluster)
    {
        clusterUpdate_ = cluster;
    }

protected:
    void plotGTCloud(const pcl::PointCloud<pcl::PointXYZL>& cloud);

    ros::NodeHandle n_;
    string maptype_, registratorType_;
    Eigen::Affine3d initPose_, sensorPose_, pose_last_fuse_;
    GraphMapNavigatorPtr graph_map_;
    GraphMapNavigatorParamPtr graph_param_;
    MapParamPtr mapParam_;
    RegParamPtr regParam_;
    RegTypePtr registrator_;
    MotionModel3d motion_model_3d_;
    plotmarker marker_;

    bool initialized_ = false;
    bool visualize_ = false;

    unsigned int nr_frames_ = 0;

    bool use_keyframe_ = true;
    double min_keyframe_dist_ = 0.5;
    double min_keyframe_rot_deg_ = 15;

    bool fuse_no_motion_frames = false;
    bool multiple_map_update = false;
    bool avoid_lidar_shadow = false;
    bool use_scanmatching = false;
    bool scanmatching_extrapolation = false;
    bool save_merged_cloud_ = false;
    bool observationInRobotFrame = false;
    bool ndt_om_ = false;
    bool use_odom_as_gt_ = false;
    bool is_lamide_ = false;
    bool only_static_ = false;
    bool clusterUpdate_ = false;
};

} // namespace graph_map
} // namespace perception_oru
#include <graph_map_lamide/graph_map_fuser_impl.h>
