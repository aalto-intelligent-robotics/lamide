#include <ndt_generic_lamide/pcl_utils.h>
// PCL specific includes
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "graph_localization_lamide/LocalisationHeatMap.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "ros/ros.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <boost/program_options.hpp>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf_conversions/tf_eigen.h>

namespace po = boost::program_options;
// using namespace std;
using namespace HeatMap;
using std::cout;
using std::endl;
using namespace perception_oru::graph_map;

double resolution;
double map_sizex, map_sizey, map_sizez;

class TestHeatmap
{
public:
    TestHeatmap(const std::string& path, double res, ros::NodeHandle* nh)
    {
        //FIXME: broken
        LoadGraphMap(path, path, graph_map);
        vis = new graphVisualization(graph_map, true, true, true);
        hmi = new HeatMapInterface(graph_map, res);
        nh_ = nh;
        sub = nh_->subscribe("/initialpose", 1, &TestHeatmap::pose_callback, this);
        cout << "initialized subscriber" << endl;
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        cout << "callback" << endl;
        Eigen::Affine3d T;
        tf::poseMsgToEigen(msg->pose.pose, T);
        MapNodePtr mptr;
        cout << "request map" << endl;
        float score = hmi->GetMapForPoint(T.translation(), mptr, graph_map);
        cout << "switched to: " << mptr->GetId() << ", score=" << score << endl;

        if (mptr != NULL)
            graph_map->SwitchToMapNode(mptr);
    }

private:
    GraphMapNavigatorPtr graph_map;
    graphVisualization* vis;
    HeatMapInterface* hmi;
    ros::Subscriber sub;
    ros::NodeHandle* nh_;
};

int main(int argc, char** argv)
{
    std::string map_path;
    ros::init(argc, argv, "test_heatmap");
    po::options_description desc("Allowed options");

    desc.add_options()("help", "produce help message")(
        "map-sizex", po::value<double>(&map_sizex)->default_value(100),
        "map size")("map-sizey", po::value<double>(&map_sizey)->default_value(100), "map size")(
        "map-sizez", po::value<double>(&map_sizez)->default_value(100),
        "map size")("map-path", po::value<string>(&map_path)->default_value(""), "path")(
        "resolution", po::value<double>(&resolution)->default_value(1.0), "reesolution of a voxel");

    // Boolean parameres are read through notifiers
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return false;
    }
    ros::NodeHandle nh("~");
    TestHeatmap(map_path, resolution, &nh);
}
