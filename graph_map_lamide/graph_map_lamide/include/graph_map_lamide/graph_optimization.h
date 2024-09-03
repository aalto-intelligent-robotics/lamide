#ifndef GRAPH_OPTIMIZATION_H
#define GRAPH_OPTIMIZATION_H
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "ndt/ndtd2d_reg_type.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "reg_type.h"
#include "ros/publisher.h"
#include "ros/ros.h"
namespace perception_oru
{
namespace graph_map
{

class graphOptimization
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    graphOptimization(GraphMapNavigatorPtr& graph_map);

    void SetFilePath(std::string& path)
    {
        output_path = path;
    }

    void SetVisualizer(graphVisualization* vis)
    {
        vis_ = vis;
    }

    void setMarker(PlotMarker marker)
    {
        marker_ = marker;
    }
    bool ConsecutiveMap2MapRegistraiton()
    {
        if (graph_map_->Size() < 2)
        {
            std::cerr << "Cannot perform map-to-map registration with (" << graph_map_->Size()
                      << ")"
                      << " nodes" << std::endl;
            return false;
        }
        if (vis_ == NULL)
        {
            cerr << "No vis exist" << endl;
            exit(0);
        }
        for (mapNodeItr itr = graph_map_->begin() + 1; itr != graph_map_->end() && ros::ok(); itr++)
        {
            vis_->PlotLinks();
            Eigen::Affine3d Tprev = (*(itr - 1))->GetPose();
            Eigen::Affine3d Tcurr = (*itr)->GetPose();
            Eigen::Affine3d Tlink = Tprev.inverse() * Tcurr;
            graph_map_->SetActiveNodes(*(itr - 1), *(itr));
            vis_->PlotActiveNodesCloud();

            double score = -1;
            bool reg_Status = reg_type->Map2MapRegistration((*(itr - 1))->GetMap(),
                                                            (*itr)->GetMap(), Tlink, score);
            if (reg_Status)
            {
                graph_map_->UpdateLink(*(itr - 1), *itr, Tlink);
                cout << "Updated link between \"" << GetNodeLinkName(*(itr - 1)) << "\" - \""
                     << GetNodeLinkName(*(itr)) << "\", score=" << score << endl;
                vis_->PlotActiveNodesCloud();
            }
            else
                cerr << "No link update" << endl;
            std::cout << "Link adjusted: " << (Tprev.inverse() * Tcurr).translation().transpose()
                      << std::endl;
        }
        graph_map_->UpdateAllNodePoses();
    }

private:
    GraphMapNavigatorPtr graph_map_;
    ros::NodeHandle nh_;
    PlotMarker marker_;
    RegTypePtr reg_type;
    graphVisualization* vis_;

    // Components for publishing
    boost::mutex m, message_m;
    std::string output_path = "";
};

class GlobalSubmapOptimisation
{

public:
    GlobalSubmapOptimisation(GraphMapNavigatorPtr& graph_map);

    double SubmapPoseScore(const Eigen::Affine3d& T1, unsigned int i);

    void ComputeAllScore();

private:
    GraphMapNavigatorPtr graph_map_;
    ros::NodeHandle nh_;
    std::vector<std::vector<Eigen::Affine3d>> pose_est_;
    std::vector<std::vector<Eigen::Affine3d>> pose_gt_;
    std::vector<MapTypePtr> maps_;
    Eigen::MatrixXd C_inv;
    double sigma = 1;
    double alpha = 1.0;
};

} // namespace graph_map
} // namespace perception_oru
#endif // GRAPH_OPTIMIZATION_H
