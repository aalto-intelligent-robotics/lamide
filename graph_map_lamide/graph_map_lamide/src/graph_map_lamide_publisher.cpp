#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "pcl/features/feature.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

#include <cstdio>
#include <cstring>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/lazy_grid.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/ndt_conversions.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/pointcloud_utils.h>
#include <string>

#include <std_srvs/Empty.h>

using namespace perception_oru;
using namespace graph_map;

bool exit_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ros::shutdown();
    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ndt_map_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle parameters("~");
    std::string mapFile;
    std::string mapPath;
    std::string mapTopic;
    std::string mapFrame, mapParentFrameId;
    double resolution;
    double mapRate;
    bool simple_plot;
    GraphMapNavigatorPtr graph_map_;

    ros::ServiceServer exit_server_;

    parameters.param<std::string>("map_file", mapFile, "file.jff");
    parameters.param<std::string>("map_path", mapPath, "file.jff");
    parameters.param<std::string>("map_topic", mapTopic, "/maps/map_2d_laser");
    parameters.param<std::string>("map_frame", mapFrame, "/maps/map_2d_laser");
    parameters.param<bool>("simple_plot", simple_plot, "use center points instead of elispsoids");
    parameters.param<std::string>("map_parent_frame_id", mapParentFrameId, "/world");
    parameters.param<double>("map_rate", mapRate, 0.5);

    exit_server_ = nh.advertiseService("pub_exit", &exit_callback);

    ros::Publisher map_pub = nh.advertise<ndt_map_lamide::NDTMapRGBMsg>(mapTopic, 1);
    ros::Rate loop_rate(mapRate);
    ndt_map_lamide::NDTMapRGBMsg msg;

    LoadGraphMap(mapPath, mapFile, graph_map_);
    graph_map::MapTypePtr map_type = graph_map_->GetCurrentMapNode();
    graphVisualization vis(graph_map_, true);
    if (simple_plot)
        vis.SetMarkerType(plotmarker::point);

    vis.SetParentFrameId(mapParentFrameId);

    /*NDTMapPtr ptr=boost::dynamic_pointer_cast<NDTMapType>(map_type);
    perception_oru::NDTMap *ndt_ptr = ptr->GetNDTMap();
    tf::TransformBroadcaster br;
    tf::Transform tf_map;
    tf::poseEigenToTF(graph_map_->GetCurrentNodePose(), tf_map);
    perception_oru::toRGBMessage(ndt_ptr, msg, mapFrame);*/

    while (ros::ok())
    {
        // map_pub.publish(msg);
        // br.sendTransform(tf::StampedTransform(tf_map, msg.header.stamp, mapParentFrameId,
        // mapFrame));
        // GraphPlot::PlotMap(map_type,-1,graph_map_->GetCurrentNodePose(),plotmarker::point);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
