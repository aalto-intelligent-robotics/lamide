
#include "boost/serialization/serialization.hpp"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graph_optimization.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"

#include <Eigen/Eigen>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/program_options.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

namespace po = boost::program_options;
using namespace perception_oru;
using namespace graph_map;
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */

int main(int argc, char** argv)
{
    string file_name;
    string output_filename;
    GraphMapNavigatorPtr graph_map;

    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")(
        "file-name",
        po::value<std::string>(&file_name)
            ->default_value(std::string("full_map_serialization.dat")),
        "name of file to load containing graphMap")(
        "output-filepath",
        po::value<std::string>(&output_filename)
            ->default_value(std::string("/home/submap_metadata")),
        "name of file to load containing graphMap");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        exit(0);
    }
    ros::init(argc, argv, "show_map");
    ros::NodeHandle param("~");
    cout << "Attempt to open map: " << file_name << endl;

    ros::Rate loop_rate(1);
    //FIXME: broken
    LoadGraphMap(file_name, file_name, graph_map);
    cout << graph_map->ToString() << endl;
    for (mapNodeItr itr = graph_map->begin(); itr != graph_map->end(); itr++)
    {
        (*itr)->GetMap()->DownSampleCloud();
    }
    graphVisualization graph_viz(graph_map, true, true, false);
    graph_viz.PlotAllClouds();
    sleep(1);
    graph_viz.PlotTrajectory();
    graph_viz.PlotAllClouds();
    graph_viz.PlotTrajectory();
    graphOptimization graph_opt(graph_map);
    graph_opt.SetVisualizer(&graph_viz);
    graph_opt.ConsecutiveMap2MapRegistraiton();
    graph_viz.PlotLinks();
    graph_viz.PlotAllClouds();

    // SaveGraphMapPLY("test.ply",graph_map);

    int i = 1;
    while (param.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        graph_viz.PlotLinks();
        graph_viz.PlotTrajectory();
    }
    return 0;
}
