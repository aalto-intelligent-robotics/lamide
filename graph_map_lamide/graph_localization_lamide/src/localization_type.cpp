#include "graph_localization_lamide/localization_type.h"
namespace perception_oru
{
namespace graph_localization
{
std::string LocalizationType::fix_frame_id = "/world";

LocalizationType::~LocalizationType()
{
}
LocalizationType::LocalizationType(LocalisationParamPtr param)
{
    if (param->graph_map_ == NULL)
    {
        cerr << "graph_map not loaded" << endl;
    }
    initialized_ = false;
    param_ = param;
    graph_map_ = param->graph_map_;
    visualize_ = param->visualize;
    localisation2D_ = param->localisation2D;
    enable_localisation_ = param->enable_localisation;
    graph_map_->SetMapSwitchMethod(param->switch_map_method);
    sensor_pose_ = param->sensor_pose;
    n_obs_search_ = param->n_obs_search;
    min_keyframe_dist_ = param->min_keyframe_dist;
    min_keyframe_dist_rot_deg_ = param->min_keyframe_dist_rot_deg;
}

std::string LocalizationType::ToString()
{
    std::stringstream ss;
    ss << "Localisation Type:" << endl;
    ss << "Initialized: " << std::boolalpha << initialized_ << endl;
    ss << graph_map_->ToString();
    return ss.str();
}
void LocalizationType::SetPose(const Eigen::Affine3d& pose)
{
    static bool first_run = true;
    double alpha = 0.7;
    if (first_run)
    {
        prev_pose_ = pose;
        pose_ = pose;
        first_run = false;
    }
    else
    {
        prev_pose_ = pose_;
        pose_ = pose;
        // velocity_=(Eigen::Scaling(alpha)*GetVelocity())*(Eigen::Scaling(1-alpha)*velocity_);
    }
}

Eigen::Affine3d LocalizationType::GetVelocity()
{
    double dt = 1.0; // time equal to frame step
    return prev_pose_.inverse() * pose_ * Eigen::Scaling(dt);
}
void LocalisationParam::GetParamFromRos()
{
    cout << "Read localisation_type parameters from ROS" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("localisation2D", localisation2D, true);
    nh.param("enable_localisation", enable_localisation, true);
    nh.param("visualize", visualize, true);
}
std::string LocalisationParam::ToString()
{
    stringstream ss;
    ss << "enable_localisation_=" << std::boolalpha << enable_localisation << endl;
    ss << "localisation2D_=" << std::boolalpha << localisation2D << endl;
    ss << "visualize=" << std::boolalpha << visualize << endl;
    return ss.str();
}
LocalisationParam::~LocalisationParam()
{
}
LocalisationParam::LocalisationParam()
{
}

} // namespace graph_localization
} // namespace perception_oru
