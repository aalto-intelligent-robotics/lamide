#include "graph_map_lamide/factor.h"
namespace perception_oru
{
namespace graph_map
{
bool factor::Connects(NodePtr node)
{
    if (node->GetId() == prev_->GetId() || node->GetId() == next_->GetId())
    {
        return true;
    }
    else
        return false;
}
factor::factor()
{
    diff_ = Eigen::Affine3d::Identity();
    prev_ = NULL;
    next_ = NULL;
    covar_ = unit_covar;
}

Eigen::Affine3d factor::GetTransform(NodePtr prev, NodePtr next, bool& status)
{
    status = true;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    if (prev == prev_ && next == next_)
        T = diff_;
    else if (prev == next_ && next == prev_)
        T = diff_.inverse();
    else
        status = false;
    return T;
}

factor::factor(MapNodePtr prev, NodePtr next, const Eigen::Affine3d& diff, const Matrix6d& cov)
{

    prev_ = prev;
    next_ = next;
    diff_ = diff;
    covar_ = cov;
}

} // namespace graph_map
} // namespace perception_oru
