#include "graph_map_lamide/visualization/graph_visualization.h"

#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/visualization/graph_plot.h"

namespace perception_oru
{
namespace graph_map
{

graphVisualization::graphVisualization(GraphMapNavigatorPtr& graph_map,
                                       bool visualization_enabled,
                                       bool visualize_map,
                                       bool parallel_execution,
                                       int color)
    : nh_("~")
    , color_(color)
{

    graph_map_ = graph_map;
    initPublishers();
    T_map = 15.0;
    T_other = 1.0;
    visualization_enabled_ = visualization_enabled;
    visualize_map_ = visualize_map;
    cluster_marker_vis = new marker_vis();
    if (visualization_enabled && parallel_execution)
    {
        plot_thread = new std::thread(&graphVisualization::PlotGraphThread, this);
    }
}

graphVisualization::graphVisualization(GraphMapNavigatorPtr& graph_map,
                                       bool parallel_execution,
                                       int color)
    : nh_("~")
    , color_(color)
{

    graph_map_ = graph_map;
    initPublishers();
    GetParametersFromRos();
    cluster_marker_vis = new marker_vis();
    if (visualization_enabled_ && parallel_execution)
    {
        plot_thread = new std::thread(&graphVisualization::PlotGraphThread, this);
    }
}

void graphVisualization::initPublishers()
{
    cloud_world_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZL>>(CLOUD_MAP, 15);
    cloud_current_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZL>>(CLOUD_CURRENT, 10);
    cloud_prev_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZL>>(CLOUD_PREV, 10);
    trajectory_pub_ = nh_.advertise<geometry_msgs::PoseArray>(TRAJECTORY_TOPIC, 10);
    est_trajectory_pub_ = nh_.advertise<geometry_msgs::PoseArray>(EST_TRAJECTORY_TOPIC, 10);
    ext_trajectory_pub_ = nh_.advertise<geometry_msgs::PoseArray>(EXT_TRAJECTORY_TOPIC, 10);
    current_trajectory_pub_ = nh_.advertise<geometry_msgs::PoseArray>(CURRENT_TRAJECTORY_TOPIC, 10);
    map_pub_ = nh_.advertise<ndt_map_lamide::NDTMapRGBMsg>(NDT_OM_MAP_TOPIC, 1);
    graph_info_pub_ = nh_.advertise<visualization_msgs::Marker>("graph_info", 10);
    occ_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(OCCUPANCY_TOPIC, 10);
}

void graphVisualization::PlotCurrentMap(PlotMarker marker, const ros::Time t)
{
    MapNodePtr map_node = graph_map_->GetCurrentNode();
    NDTDLMapPtr DLMapPtr = boost::dynamic_pointer_cast<NDTDLMapType>(map_node->GetMap());
    NDTMapPtr NDTMapPtr = boost::dynamic_pointer_cast<NDTMapType>(map_node->GetMap());
    if (DLMapPtr != NULL || NDTMapPtr != NULL)
    {
        NDTMap* ndt_map;
        if (DLMapPtr != NULL)
        {
            std::vector<NDTMap*> maps = DLMapPtr->GetNDTMaps();
            std::string topic = "ndtdl";
            Eigen::Affine3d T;
            T.setIdentity();
            for (size_t j = 0; j < maps.size(); j++)
            {
                std::string tmp = topic + std::to_string(j);
                std::cout << "PlotNDT : " << tmp << std::endl;
                GraphPlot::PlotNDTRGB(maps[j], GetNodeLinkName(map_node), tmp, T);
                std::cout << "PlotNDT - done" << std::endl;
            }
        }
        else if (NDTMapPtr != NULL)
        {
            if (marker == plotmarker::point)
            {
                GraphPlot::PlotMap(NDTMapPtr, -1, map_node->GetPose(), plotmarker::point);
            }
            else
            {
                ndt_map = NDTMapPtr->GetNDTMap();
                ndt_map_lamide::NDTMapRGBMsg msg;
                string link_id = GetNodeLinkName(map_node);
                toRGBMessage(ndt_map, msg, link_id, color_);
                msg.header.stamp = t;
                map_pub_.publish(msg);
            }
        }
    }
    else
    {
        std::cerr << "graphVisualization::PlotCurrentMap don't handle this maptype" << std::endl;
        return;
    }
}

void graphVisualization::PlotOccupancyMap(const ros::Time t)
{
    nav_msgs::OccupancyGrid occ_msg;
    occ_msg.header.stamp = t;
    MapNodePtr map_node = graph_map_->GetCurrentNode();
    NDTMapPtr NDTMapPtr = boost::dynamic_pointer_cast<NDTMapType>(map_node->GetMap());
    if (NDTMapPtr != NULL)
    {
        NDTMap* ndt_map = NDTMapPtr->GetNDTMap();
        string link_id = GetNodeLinkName(map_node);
        toOccupancyGrid(ndt_map, occ_msg, 0.5, link_id);
        occ_msg.header.frame_id = link_id;
        occ_pub_.publish(occ_msg);
    }
}

void graphVisualization::PlotLinks(int currentId, ros::Time t)
{
    std::string prev_frame_id = parent_frame_id;
    current_id_ = currentId;

    for (mapNodeItr itr = graph_map_->begin(); itr != graph_map_->end(); itr++)
    {
        tf::Transform node_tf;
        Eigen::Affine3d diff;
        if (itr == graph_map_->begin())
        {
            // get absolyute pose of the first node
            diff = (*itr)->GetPose();
        }
        else
        {
            bool status = false;
            FactorPtr f = NULL;
            MapNodePtr prev = NULL;
            for (mapNodeItr itrj = graph_map_->begin(); itrj != itr; itrj++)
            {
                prev = *itrj;
                f = graph_map_->FindLink(prev, *itr);

                if (f != NULL)
                    break;
            }
            if (f == NULL)
            {
                cerr << "Cannot find link for plotting" << endl;
                return;
            }
            prev_frame_id = GetNodeLinkName(prev);
            diff = f->GetTransform(prev, *itr, status); // get relative transformation between nodes
            if (!status)
                cerr << "Transformation not found netween LINKS"
                     << endl; // \""<<GetNodeLinkName(*(itr-1))<<"\" and
                              // \""<<GetNodeLinkName(*itr)<<"\""<<endl;
        }
        tf::poseEigenToTF(diff, node_tf);
        std::string frame_id = GetNodeLinkName(*itr);
        trans_pub_.sendTransform(tf::StampedTransform(node_tf, t, prev_frame_id, frame_id));
        prev_frame_id = frame_id;

        int idx = (*itr)->GetId();
        if (idx == currentId)
        {
            tf::Transform notf;
            tf::poseEigenToTF(Eigen::Affine3d::Identity(), notf);
            trans_pub_.sendTransform(tf::StampedTransform(notf, t, frame_id, "CurrentNode"));
        }
    }
}

void graphVisualization::PlotCurrentLinkInfo(ros::Time t)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = GetNodeLinkName(graph_map_->GetCurrentNode());

    marker.header.stamp = t;
    marker.ns = "graphInfo";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(100);
    marker.scale.z = 0.2;
    marker.color.a = 1;
    marker.color.b = 1;

    marker.pose.position.z = 1.0;

    std::ostringstream os;
    os << std::setw(3) << "(x,y)=(" << marker.pose.position.x << "," << marker.pose.position.y
       << "), id=" << graph_map_->GetCurrentNode()->GetId() << endl;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.text = os.str();
    graph_info_pub_.publish(marker);
}

void graphVisualization::PlotAllClouds()
{
    pcl::PointCloud<pcl::PointXYZL> cloud;
    graph_map_->GetCloud(cloud, true);
    cloud.header.frame_id = parent_frame_id;
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud_world_pub.publish(cloud);
}

void graphVisualization::PlotActiveNodesCloud()
{
    ros::Time t_plot = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZL> cloud_prev, cloud_curr;
    MapNodePtr curr = graph_map_->GetCurrentNode();
    graph_map_->GetCurrentMapNode()->GetCloud(cloud_curr, true);
    pcl_conversions::toPCL(t_plot, cloud_curr.header.stamp);
    cloud_curr.header.frame_id = GetNodeLinkName(curr);
    cloud_current_pub_.publish(cloud_curr);
    MapNodePtr prev = graph_map_->GetPreviousNode();
    if (prev != NULL && prev != curr)
    {
        if (not prev->isOnDisk())
        {
            prev->GetMap()->GetCloud(cloud_prev, true);
            pcl_conversions::toPCL(t_plot, cloud_prev.header.stamp);
            cloud_prev_pub_.publish(cloud_prev);
            cloud_prev.header.frame_id = GetNodeLinkName(prev);
        }
    }
}

void graphVisualization::PlotTrajectory(ros::Time t)
{
    Eigen::Affine3d Tsensor = graph_map_->GetSensorPose();
    geometry_msgs::PoseArray pose_arr;
    geometry_msgs::Pose pose;

    pose_arr.header.frame_id = parent_frame_id;
    pose_arr.header.stamp = t;
    bool created = false;
    for (mapNodeItr itr = graph_map_->begin(); itr != graph_map_->end(); itr++)
    {
        if ((*itr) == NULL)
        {
            continue;
        }
        if ((*itr)->isOnDisk())
        {
            continue;
        }
        MapTypePtr map = (*itr)->GetMap();
        if (map == NULL)
        {
            continue;
        }
        ndt_generic::Affine3dSTLVek vek = map->GetObservationVector();
        for (int i = 0; i < vek.size(); i++)
        {
            tf::poseEigenToMsg((*itr)->GetPose() * vek[i], pose);
            pose_arr.poses.push_back(pose);
        }
        created = true;
    }
    if (created)
    {
        trajectory_pub_.publish(pose_arr);
    }
}

void graphVisualization::PlotClusteredTrajectory()
{
    if (cluster_marker_vis != NULL && visualization_enabled_)
    {
        std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses;
        std::vector<unsigned int> idx;
        for (mapNodeItr itr = graph_map_->begin(); itr != graph_map_->end(); itr++)
        {
            if ((*itr)->isOnDisk())
            {
                continue;
            }
            MapTypePtr map = (*itr)->GetMap();
            if (map == NULL)
            {
                continue;
            }
            ndt_generic::Affine3dSTLVek vek = map->GetObservationVector();
            for (int i = 0; i < vek.size(); i++)
            {
                Eigen::Affine3d pose = (*itr)->GetPose() * vek[i];
                poses.push_back(pose);
                idx.push_back((*itr)->GetId());
            }
        }
        cluster_marker_vis->PlotLabeledPoses(poses, idx);
    }
}

void graphVisualization::PlotCurrentMapTrajectory(ros::Time t)
{
    geometry_msgs::PoseArray pose_arr;
    geometry_msgs::Pose pose;

    pose_arr.header.frame_id = parent_frame_id;
    pose_arr.header.stamp = t;
    MapNodePtr node_ptr = graph_map_->GetCurrentNode();
    MapTypePtr mapNode = graph_map_->GetCurrentMapNode();
    if (node_ptr == NULL || mapNode == NULL)
    {
        return;
    }
    ndt_generic::Affine3dSTLVek vek = mapNode->GetObservationVector();
    for (int i = 0; i < vek.size(); i++)
    {
        tf::poseEigenToMsg(node_ptr->GetPose() * vek[i], pose);
        pose_arr.poses.push_back(pose);
    }
    if (pose_arr.poses.size() > 0)
    {
        current_trajectory_pub_.publish(pose_arr);
    }
}

void graphVisualization::PlotEstGtTrajectory(ros::Time t)
{
    Eigen::Affine3d Tsensor = graph_map_->GetSensorPose();
    geometry_msgs::PoseArray pose_arr, pose_ext_arr;
    geometry_msgs::Pose pose;

    pose_arr.header.frame_id = pose_ext_arr.header.frame_id = parent_frame_id;
    pose_arr.header.stamp = pose_ext_arr.header.stamp = t;

    for (mapNodeItr itr = graph_map_->begin(); itr != graph_map_->end(); itr++)
    {
        if ((*itr)->isOnDisk())
        {
            continue;
        }
        std::vector<Eigen::Affine3d> poseFrames, poseFramesExt;
        poseFrames = (*itr)->GetPoseEst();
        poseFramesExt = (*itr)->GetPoseExt();
        if (poseFrames.size() != poseFramesExt.size())
        {
            std::cerr << "pose not equal" << std::endl;
            continue;
        }

        for (int i = 0; i < poseFrames.size(); i++)
        {
            Eigen::Affine3d glob_poseFrame = (*itr)->GetPose() * poseFrames[i];
            tf::poseEigenToMsg(glob_poseFrame, pose);
            pose_arr.poses.push_back(pose);
            tf::poseEigenToMsg(poseFramesExt[i], pose);
            pose_ext_arr.poses.push_back(pose);
        }
    }
    if (pose_arr.poses.size() > 0)
        ext_trajectory_pub_.publish(pose_arr);
    if (pose_ext_arr.poses.size() > 0)
        est_trajectory_pub_.publish(pose_ext_arr);
}

void graphVisualization::PlotGraphThread()
{
    // FIXME: to something sensible?
    // Now the problem is with sim time, ros::time::now = 0
    // so when subtracted, the time diff becomes negative and crashes
    ros::Time tmap_last;
    ros::Time tother_last;
    try
    {
        tmap_last = ros::Time::now() - ros::Duration(T_map);
        tother_last = ros::Time::now() - ros::Duration(T_other);
    }
    catch (const std::exception& e)
    {
        tmap_last = ros::Time::now();
        tother_last = ros::Time::now();
    }

    usleep(1000 * 100);
    while (ros::ok() && !despawn_thread_)
    {
        ros::Time tnow = ros::Time::now();
        ros::Duration td;
        ros::Duration td2;
        try
        {
            td = tnow - tother_last;
            td2 = tnow - tmap_last;
        }
        catch (const std::exception& e)
        {
            td = tother_last - tnow;
            td2 = tmap_last - tnow;
        }

        if (visualization_enabled_ && (td > ros::Duration(T_other) || force_update_))
        {
            graph_map_->m_graph.lock();
            PlotLinks(current_id_, tnow);
            PlotCurrentLinkInfo(tnow);
            PlotTrajectory(tnow);
            PlotEstGtTrajectory(tnow);
            PlotCurrentMapTrajectory(tnow);
            graph_map_->m_graph.unlock();
            force_update_ = false;
            tother_last = tnow;
        }
        if (visualize_map_ && visualization_enabled_ &&
            (td2 > ros::Duration(T_map) || force_update_))
        {

            graph_map_->m_graph.lock();
            PlotLinks(current_id_, tnow);
            PlotCurrentLinkInfo(tnow);
            PlotTrajectory(tnow);
            PlotEstGtTrajectory(tnow);
            PlotCurrentMap(marker_, tnow);
            PlotClusteredTrajectory();
            graph_map_->m_graph.unlock();
            force_update_ = false;
            tmap_last = tnow;
        }
        usleep(10000);
    }
}

void graphVisualization::GetParametersFromRos()
{
    nh_.param<double>("T_map", T_map, 5.0);
    nh_.param<double>("T_other", T_other, 1.0);
    nh_.param("visualize", visualization_enabled_, true);
    bool disable_map;
    nh_.param("disable_map_visualization", disable_map, false);
    visualize_map_ = !disable_map;
    // cout << "Visualize :" << std::boolalpha << visualization_enabled_
    //      << ", visualize map: " << std::boolalpha << visualize_map_
    //      << ", Map display period: " << T_map << ", Link display period: " << T_other << endl;
}
} // namespace graph_map
} // namespace perception_oru
