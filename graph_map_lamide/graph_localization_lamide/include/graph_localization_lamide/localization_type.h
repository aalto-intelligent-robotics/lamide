#ifndef LOCALISATIONTYPE_H
#define LOCALISATIONTYPE_H
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <ndt_generic_lamide/point_types.h>

namespace perception_oru
{
namespace graph_localization
{
using namespace graph_map;

class LocalizationType
{
public:
    LocalizationType(LocalisationParamPtr param);

    virtual ~LocalizationType() = 0;

    virtual std::string ToString();

    virtual void InitializeLocalization(
        const Eigen::Affine3d& pose,
        const Vector6d& variance) = 0; // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    virtual bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor) = 0;

    virtual bool UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor) = 0;

    void SetPose(const Eigen::Affine3d& pose);

    Eigen::Affine3d GetPose()
    {
        return pose_;
    } // return pose of robot in world frame

    Eigen::Affine3d GetVelocity();

    void visualize(bool viz)
    {
        visualize_ = viz;
    }

    bool Initialized()
    {
        return initialized_;
    }

    virtual unsigned int GetFramesCount()
    {
        return 0;
    }

    static std::string fix_frame_id;

    // note debug
    virtual void ColorVoxelsWithScore(std::vector<perception_oru::NDTCell*> ndts){};

    virtual std::vector<std::string> logPointHistogram(const std::string& path,
                                                       const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                                       const Eigen::Affine3d& pose,
                                                       int node_id){};

    virtual void logMatchHistogram(const std::vector<perception_oru::NDTCell*>& ndts){};

    virtual std::vector<std::string> getHistogramStrings(){};
    virtual void setCreateHistogram(bool set, int ratio = -1){};
    virtual void setDisplayErrorMap(bool setError, bool setLocal){};
    virtual void setDistributionError(bool set){};
    virtual void SetMapSwitchMethod(MapSwitchingMethod method){};
    virtual void SetMapInterchangeRadius(double radius){};
    virtual MapSwitchingMethod GetMapSwitchMethod() const {};
    virtual perception_oru::NDTMap& getLocalMap(bool& success){};

protected:
    bool initialized_ = false;
    bool visualize_ = false;
    bool localisation2D_ = true;
    bool enable_localisation_ = true;
    bool no_motion_localisaton_disable = false;
    unsigned int n_obs_search_ = 5;
    double min_keyframe_dist_ = 0.01;
    double min_keyframe_dist_rot_deg_ = 100;
    Eigen::Affine3d sensor_pose_;
    GraphMapNavigatorPtr graph_map_;
    LocalisationParamPtr
        param_; // holds the parameters for localisation, can be used to initialize or Reinitialize

    Eigen::Affine3d pose_ = Eigen::Affine3d::Identity();
    Eigen::Affine3d prev_pose_ = Eigen::Affine3d::Identity();
    Eigen::Affine3d velocity_ = Eigen::Affine3d::Identity();
    friend class LocalisationFactory;
};

class LocalisationParam
{
public:
    LocalisationParam();

    ~LocalisationParam();

    virtual void GetParamFromRos();

    virtual std::string ToString();

    GraphMapNavigatorPtr graph_map_;
    bool visualize = false;
    bool enable_measurments = false;
    bool localisation2D = true;
    bool enable_localisation = true;
    unsigned int n_obs_search = 5;
    double min_keyframe_dist = 0.01;
    double min_keyframe_dist_rot_deg = 100;
    MapSwitchingMethod switch_map_method = node_position;
    Eigen::Affine3d sensor_pose;

private:
    friend class LocalisationFactory;
};

} // namespace graph_localization

} // namespace perception_oru
#endif // LOCALISATIONTYPE_H
