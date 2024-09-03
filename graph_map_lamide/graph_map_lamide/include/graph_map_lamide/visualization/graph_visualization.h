#ifndef GRAPH_VISUALIZATION_H
#define GRAPH_VISUALIZATION_H
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/map_node.h"
#include "graph_map_lamide/ndt/ndt_map_type.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "graph_map_lamide/visualization/marker_vis.h"
#include "nav_msgs/Path.h"
#include "ndt_map_lamide/NDTMapMsg.h"
#include "ndt_map_lamide/ndt_conversions.h"
#include "ndt_map_lamide/ndt_map.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "string.h"
#include "tf_conversions/tf_eigen.h"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

namespace perception_oru
{
namespace graph_map
{

class graphVisualization
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    graphVisualization(GraphMapNavigatorPtr& graph_map,
                       bool visualization_enabled,
                       bool visualize_map,
                       bool parallel_execution,
                       int color = 1);

    graphVisualization(GraphMapNavigatorPtr& graph_map,
                       bool parallel_execution,
                       int color = 1); // gets parameters from ros

    ~graphVisualization()
    {
        despawn_thread_ = true;
        std::thread(*plot_thread);
    }

    void SetParameters(double Tmap, double Tother)
    {
        T_map = Tmap;
        T_other = Tother;
    }

    void PlotCurrentMap(PlotMarker marker = sphere, const ros::Time t = ros::Time::now());

    void PlotTrajectory(ros::Time t = ros::Time::now());

    void PlotCurrentMapTrajectory(ros::Time t = ros::Time::now());

    void PlotEstGtTrajectory(ros::Time t);

    void PlotActiveNodesCloud();

    void PlotAllClouds();

    void PlotLinks(int currentId = 0, ros::Time t = ros::Time::now());

    void PlotCurrentLinkInfo(ros::Time t = ros::Time::now());

    void PlotOccupancyMap(ros::Time t = ros::Time::now());

    void ForcePlot()
    {
        force_update_ = true;
    }

    void GetParametersFromRos();

    void PlotClusteredTrajectory();

    void SetParentFrameId(const std::string& frame)
    {
        parent_frame_id = frame;
    }

    void SetMarkerType(plotmarker marker)
    {
        marker_ = marker;
    }

    void setColor(int color)
    {
        color_ = color;
    }

private:
    void initPublishers();

    void PlotGraphThread();

    ros::Publisher cloud_world_pub, cloud_current_pub_, cloud_prev_pub_;
    ros::Publisher trajectory_pub_, current_trajectory_pub_, ext_trajectory_pub_,
        est_trajectory_pub_, observation_pub_;
    ros::Publisher map_pub_, occ_pub_;
    ros::Publisher graph_info_pub_;
    tf::TransformBroadcaster submap_pub_;
    tf::TransformBroadcaster trans_pub_;
    std::string tf_prefix_ = "/tf";
    ros::NodeHandle nh_;
    GraphMapNavigatorPtr graph_map_;
    marker_vis* cluster_marker_vis;
    plotmarker marker_ = plotmarker::sphere;

    bool visualization_enabled_ = false;
    bool visualize_map_ = false;
    double T_map = 5.0, T_other = 1.0;
    bool force_update_ = false;
    std::thread* plot_thread;
    bool despawn_thread_ = false;
    std::string parent_frame_id = "/world";
    int current_id_ = 0;
    int color_ = 1;
};

} // namespace graph_map
} // namespace perception_oru
#endif // GRAPH_VISUALIZATION_H
