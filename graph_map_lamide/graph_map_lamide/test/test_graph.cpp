#include "Eigen/Geometry"
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "stdio.h"

#include <pcl/point_cloud.h>

using namespace std;
using namespace Eigen;

namespace po = perception_oru;
namespace pogm = perception_oru::graph_map;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testGraphLib");
    ros::NodeHandle n;

    string maptype = "ndt_map";

    //   n.param<std::string>("map_type",maptype,"ndt_map");
    cout << "Testing graph_map with map type: " << maptype << endl;

    Eigen::Affine3d initPose = Affine3d::Identity();
    initPose.translation() << 2.5, 2.5, 0.01; // Create initiali pose of graph

    Eigen::Affine3d diff = Affine3d::Identity();

    diff = AngleAxisd(0.0 * M_PI, Vector3d::UnitX()) * AngleAxisd(0.0 * M_PI, Vector3d::UnitY()) *
           AngleAxisd(0.2 * M_PI, Vector3d::UnitZ()) *
           Translation3d(1, 1, 0); // Transformation between subsequent map nodes

    pogm::Matrix6d cov;
    Eigen::DiagonalMatrix<double, 6> diag1;
    diag1.diagonal() << 0.15, 0.15, 0.15, 0.01, 0.01, 0.01;
    cov = diag1; // Create covariance to represent uncertainty betweem mpde

    pogm::MapParamPtr mapParam_ = pogm::GraphFactory::CreateMapParam(maptype);

    std::cout << "createddddddd map param" << std::endl;
    //
    pogm::GraphMapNavigatorParamPtr graph_param =
        pogm::GraphMapNavigatorParamPtr(new pogm::GraphMapNavigatorParam());
    std::cout << "createddddddd map navigator param" << std::endl;
    pogm::GraphMapNavigatorPtr graph =
        pogm::GraphMapNavigatorPtr(new pogm::GraphMapNavigator(initPose, mapParam_, graph_param));
    cout << "size of graph :" << graph->Size() << endl;
}
