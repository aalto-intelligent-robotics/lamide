#include "graph_localization_lamide/pose_queue.h"
namespace perception_oru
{
namespace graph_localization
{

PoseQueue::PoseQueue(double max_duration, size_t max_elem)
{
    max_duration_ = max_duration;
    max_elem_ = max_elem;
}
void PoseQueue::push(const Eigen::Affine3d& pose, const double t_stamp)
{
    m.lock();
    poses_.push_back(pose);
    t_stamp_.push_back(t_stamp);
    while (fabs(t_stamp - t_stamp_[0]) > max_duration_ || poses_.size() > max_elem_)
    {
        // std::cout<<"Remove old pose ["<< poses_[0].translation().transpose()<<"]
        // t="<<t_stamp_[0]<<", time difference="<<fabs(t_stamp-t_stamp_[0])<<std::endl;
        poses_.erase(poses_.begin());
        t_stamp_.erase(t_stamp_.begin());
    }
    m.unlock();
}

void PoseQueue::push(geometry_msgs::PoseStamped pose)
{ // get pose at time t
    Eigen::Affine3d p;
    pose.header.stamp.toSec();
    tf::poseMsgToEigen(pose.pose, p);
    push(p, pose.header.stamp.toSec());
}
bool PoseQueue::PoseAvaliable(const double t_stamp)
{
    m.lock();
    if (t_stamp_.size() < 1)
    {
        m.unlock();
        return false;
    }
    if (!(t_stamp < t_stamp_[0]) && !(t_stamp > t_stamp_[t_stamp_.size() - 1]))
    { // extrapolate backward
        m.unlock();
        return true;
    }
    else
    {
        m.unlock();
        return false;
    }
}

bool PoseQueue::GetPose(const double t_stamp, Eigen::Affine3d& pose)
{ // get pose at time t

    if (!PoseAvaliable(t_stamp))
    {
        return false;
    }
    m.lock();
    for (int i = t_stamp_.size() - 2; i >= 0; i--)
    {
        if ((t_stamp_[i] < t_stamp || ndt_generic::AlmostEqual(t_stamp_[i], t_stamp)) &&
            (t_stamp < t_stamp_[i + 1] || ndt_generic::AlmostEqual(t_stamp_[i + 1], t_stamp)))
        {
            double factor = (t_stamp - t_stamp_[i]) / (t_stamp_[i + 1] - t_stamp_[i]);
            pose = ndt_generic::Interpolate(poses_[i], poses_[i + 1], factor);
            m.unlock();
            return true;
        }
    }
    m.unlock();
    std::cerr << "time outside boundries" << std::endl;
    return false;
}

void PoseQueue::ToString()
{
    m.lock();
    std::cout << "size: " << t_stamp_.size() << ", max elem=" << max_elem_
              << ", max duration=" << max_duration_ << std::endl;
    for (int i = 0; i < poses_.size(); i++)
        std::cout << "[" << poses_[i].translation().transpose() << "]"
                  << " t=" << t_stamp_[i] << std::endl;
    m.unlock();
}

} // namespace graph_localization
} // namespace perception_oru
