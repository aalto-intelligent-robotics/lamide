#include "graph_map_lamide/register.h"
namespace perception_oru
{
namespace graph_map
{

Registration::Registration(const std::string& maptype, const std::string& registratorType)
    : nh_("~")
{
    regParam_ = GraphFactory::CreateRegParam(registratorType);
    cout << "started reading reg par from ros" << endl;
    regParam_->GetParametersFromRos();
    cout << "finished reading reg par from ros" << endl;
    cout << "started reading map par from ros" << endl;

    cout << "time to create graph inside fuser" << endl;
    regParam_->sensor_pose = Eigen::Affine3d::Identity();
    registrator_ = GraphFactory::CreateRegistrationType(regParam_);
    cout << "Registraiton parameters: " << registrator_->ToString() << endl;

    nh_.param<bool>("use_keyframe", use_keyframe_, false);
    nh_.param<double>("min_keyframe_dist", min_keyframe_dist_, 0.5);
    nh_.param<double>("min_keyframe_rot_deg", min_keyframe_rot_deg_, 5);
}

bool Registration::KeyFrameBasedFuse(const Eigen::Affine3d& Tdiff)
{
    if (!use_keyframe_)
        return true;

    Eigen::Vector3d Tmotion_euler = Tdiff.rotation().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(Tmotion_euler);

    if (Tdiff.translation().norm() > min_keyframe_dist_ ||
        Tmotion_euler.norm() > (min_keyframe_rot_deg_ * M_PI / 180.0))
        return true;
    else
        return false;
}

} // namespace graph_map

} // namespace perception_oru
