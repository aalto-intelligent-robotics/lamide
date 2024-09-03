#include "graph_map_lamide/graph_optimization.h"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_vector.h>

namespace perception_oru
{
namespace graph_map
{
graphOptimization::graphOptimization(GraphMapNavigatorPtr& graph_map) : nh_("~")
{
    marker_ = plotmarker::point;
    graph_map_ = graph_map;
    NDTD2DRegParamPtr par = NDTD2DRegParamPtr(new NDTD2DRegParam());
    par->registration2d = false;
    par->matcher2D_ITR_MAX = 100;
    reg_type = GraphFactory::CreateRegistrationType(par);
}

GlobalSubmapOptimisation::GlobalSubmapOptimisation(GraphMapNavigatorPtr& graph_map) : nh_("~")
{
    graph_map_ = graph_map;

    for (mapNodeItr itr = graph_map->begin(); itr != graph_map->end(); itr++)
    {
        pose_est_.push_back((*itr)->GetPoseEst());
        pose_gt_.push_back((*itr)->GetPoseExt());
        maps_.push_back((*itr)->GetMap());
    }
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix<double, 1, 6> var;
    var << 0.06, 0.06, 3, 0.02, 0.02, 0.02;
    M.diagonal() = var;
    C_inv = M.inverse();
}
double GlobalSubmapOptimisation::SubmapPoseScore(const Eigen::Affine3d& T1, unsigned int i)
{
    if (pose_est_.size() == 0)
        return 0;

    Eigen::Matrix<double, 1, 6> t_diff;
    double score = 0;
    cout << "node " << i << endl;
    for (int j = 0; j < pose_est_.size(); j++)
    {
        Eigen::Affine3d T = T1 * pose_est_[i][j];
        Eigen::Vector3d eul_est = T.rotation().eulerAngles(0, 1, 2);
        Eigen::Vector3d eul_gt = pose_gt_[i][j].rotation().eulerAngles(0, 1, 2);

        ndt_generic::normalizeEulerAngles(eul_est);
        ndt_generic::normalizeEulerAngles(eul_gt);
        Eigen::Vector3d e_diff = (eul_est - eul_gt);
        ndt_generic::normalizeEulerAngles(e_diff);
        t_diff.block(0, 0, 1, 3) = (T.translation() - pose_gt_[i][j].translation());
        t_diff.block(0, 3, 1, 3) = e_diff;

        double tmp = t_diff * C_inv * t_diff.transpose();
        double tmp2 = exp(-tmp * tmp / 2); ///(2*sigma*sigma
        score += tmp2;
        cout << "this: " << tmp << ", exp:" << tmp2 << endl;
    }
    score = score / pose_est_.size();
    cout << ", total node:" << i << " = " << score << endl;
    return score;
}

void GlobalSubmapOptimisation::ComputeAllScore()
{
    int i = 0;
    for (mapNodeItr itr = graph_map_->begin(); itr != graph_map_->end(); itr++)
    {
        Eigen::Affine3d T = (*itr)->GetPose();
        SubmapPoseScore(T, i++);
    }
}

} // namespace graph_map
} // namespace perception_oru
