#ifndef POSE_QUEUE_H
#define POSE_QUEUE_H
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "mutex"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ros/ros.h"
#include "stdio.h"

#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
namespace perception_oru
{
namespace graph_localization
{
class PoseQueue
{

public:
    PoseQueue(double max_duration = 5.0, size_t max_elem = SIZE_MAX);

    void push(const Eigen::Affine3d& pose, const double t_stamp);

    void push(geometry_msgs::PoseStamped pose); // get pose at time t

    bool GetPose(const double t_stamp, Eigen::Affine3d& pose); // get pose at time t

    bool PoseAvaliable(const double t_stamp);

    size_t Size() const
    {
        return t_stamp_.size();
    }

    void ToString();

protected:
    double max_duration_;
    size_t max_elem_;
    std::mutex m;
    std::vector<double> t_stamp_;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses_;
};
} // namespace graph_localization

} // namespace perception_oru
#endif // POSE_QUEUE_H
