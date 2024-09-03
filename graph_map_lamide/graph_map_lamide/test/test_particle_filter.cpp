#include "Eigen/Geometry"
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "iostream"
#include "ndt_localization_lamide/3d_ndt_mcl.hpp"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "stdio.h"

#include <pcl/point_cloud.h>

using namespace std;
using namespace Eigen;

using namespace perception_oru;
using namespace graph_map;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_particle_filter");
    ros::NodeHandle n;
    string maptype;
    ros::Rate r(10);
    particle_filter_3d filter;
    cout << "created pfilter instance" << endl;
    filter.initializeNormalRandom(1000, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01);
    cout << "initialized filter" << endl;
    while (n.ok())
    {
        Eigen::Affine3d Tmotion = Eigen::Affine3d::Identity();
        Tmotion.translation() << 0.001, 0.00, 0.00;
        filter.predict(Tmotion, 0.03, 0.03, 0.03, 0.001, 0.001, 0.001);
        Eigen::Affine3d offset = Eigen::Affine3d::Identity();
        //    GraphPlot::plotParticleCloud(offset,filter.pcloud);
        cout << "position of mean" << filter.getMean().translation().transpose() << endl;
    }
}
