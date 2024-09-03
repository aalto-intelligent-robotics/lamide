#include "eigen_conversions/eigen_msg.h"
#include "graph_localization_lamide/localization_factory.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt.h"
#include "graph_localization_lamide/pose_queue.h"
#include "message_filters/subscriber.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/motionmodels.h"
#include "ndt_generic_lamide/utils.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "velodyne_msgs/VelodyneScan.h"

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <cstdio>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/ndt_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <time.h>
#include <velodyne_pointcloud_oru/rawdata.h>

#define MAX_TRANSLATION_DELTA 2.0
#define MAX_DURATION          7.0
#define MAX_ROTATION_DELTA    0.5

/** \brief A ROS node which implements NDT-MCL localisation with accurate timing and pointcloud
 * \author Daniel Adolfsson and Henrik Andreasson
 *
 */
using namespace perception_oru;
using namespace graph_map;
using namespace graph_localization;

class GraphLocalisationNode
{

protected:
    // Our NodeHandle

    Eigen::Affine3d pose_prev, pose_now;
    double t_prev, t_now;
    Eigen::Affine3d Odom_prev, odom_now;

    mutex m_odom_poses;
    PoseQueue odom_poses;

    ros::NodeHandle* nh_;
    ros::Subscriber velodyne_sub_, laser_2d_sub_, odom_sub_, gt_state_sub_, init_pose_ros_sub_;
    velodyne_rawdata::RawData velodyne_parser;
    // Components for publishing
    tf::TransformBroadcaster tf_;
    ros::Publisher output_pub_;

    Eigen::Affine3d Tsensor_;
    plotmarker plot_marker;
    unsigned int frame_nr_;
    std::string localisation_type_name;
    std::string velodyne_config_file = "";

    std::string velodyne_topic, laser_topic, map_path, odometry_topic,
        dataset_name; // dataset_name is used to aquire correct sensor_offset
    std::string world_link_id, odometry_link_id, pose_est_link, laser_link_id, init_pose_frame,
        gt_topic_, bag_name, state_base_link_id;
    double max_range, min_range, sensor_offset_t_;
    bool localisation_2d;
    bool laser_2d;
    bool useOdometry;
    bool initPoseRos;
    bool initPoseGt;
    bool initPoseSet = false;
    bool gt_localisation;
    bool visualize;
    double varz = 0.1;

    GraphMapNavigatorPtr graph_map_;
    LocalisationTypePtr localisation_type_ptr_;
    // ros::Time time_now,time_last_itr;
    ros::Publisher laser_publisher_, point2_publisher_, odom_publisher_, pose_est_publisher_;
    nav_msgs::Odometry fuser_odom;

    bool use_tf_listener_;
    Eigen::Affine3d last_tf_frame_;

    // boost::mutex m;
public:
    // Constructor
    GraphLocalisationNode(ros::NodeHandle* nh) : frame_nr_(0), odom_poses(MAX_DURATION)
    {
        nh_ = nh;

        // if we want to build map reading scans directly from bagfile

        // topic to wait for point clouds, if available
        nh_->param<std::string>("velodyne_topic", velodyne_topic, "velodyne_packets");
        // topic to wait for laser scan messages, if available
        nh_->param<std::string>("laser_topic", laser_topic, "laser_scan");

        nh_->param("init_pose_ros", initPoseRos, false);

        nh_->param("init_pose_gt", initPoseGt, true);
        nh_->param<std::string>("gt_topic", gt_topic_, "/vmc_navserver/state");

        // only match 2ith 3dof
        nh_->param("localisation_2d", localisation_2d, true);
        // enable for LaserScan message input
        nh_->param("scanner_2d", laser_2d, true);

        nh_->param<std::string>("localisation_type", localisation_type_name, "ndt_mcl");
        // range to cutoff sensor measurements

        nh_->param("max_range", max_range, 50.0);
        nh_->param("min_range", min_range, 1.0);

        // visualize in a local window
        nh_->param("visualize", visualize, true);

        std::string marker_str;
        nh_->param<std::string>("plot_marker", marker_str, "point");
        if (marker_str.compare("sphere") == 0)
            plot_marker = plotmarker::sphere;
        else if (marker_str.compare("point") == 0)
            plot_marker = plotmarker::point;
        else
            plot_marker = plotmarker::sphere;

        // if using the HMT fuser, NDT maps are saved in this directory.
        // a word of warning: if you run multiple times with the same directory,
        // the old maps are loaded automatically
        nh_->param<std::string>("map_path", map_path, "");

        Eigen::Vector3d sensor_offset_pos, sensor_offset_euler;
        // pose of the sensor with respect to the vehicle odometry frame
        nh_->param("sensor_pose_x", sensor_offset_pos(0), 0.);
        nh_->param("sensor_pose_y", sensor_offset_pos(1), 0.);
        nh_->param("sensor_pose_z", sensor_offset_pos(2), 0.);
        nh_->param("sensor_pose_r", sensor_offset_euler(0), 0.);
        nh_->param("sensor_pose_p", sensor_offset_euler(1), 0.);
        nh_->param("sensor_pose_t", sensor_offset_euler(2), 0.);
        nh_->param("sensor_offset_t", sensor_offset_t_, 0.);
        Tsensor_ = ndt_generic::vectorsToAffine3d(sensor_offset_pos, sensor_offset_euler);

        nh->param("laser_variance_z", varz, 0.25);

        // topic to wait for laser scan messages, if available
        nh_->param<std::string>("odometry_topic", odometry_topic, "/vmc_navserver/odom");
        nh_->param("use_odometry", useOdometry, true);

        nh_->param<std::string>("world_frame", world_link_id, "/world");
        nh_->param<std::string>("robot_frame_id", pose_est_link, "/pose_est_link");
        nh_->param<std::string>("laser_frame_id", laser_link_id, "/velodyne");
        nh_->param<std::string>("state_base_link_id", state_base_link_id, "/state_base_link");
        nh_->param<std::string>("velodyne_config_file", velodyne_config_file, "");

        // use standard odometry messages for initialuess

        nh_->param<bool>("use_tf_listener", use_tf_listener_, false);
        nh_->param<std::string>("odometry_frame_id", odometry_link_id,
                                std::string("/odom_base_link"));

        initPoseSet = false;
        fuser_odom.header.frame_id = "/world";

        laser_publisher_ = nh_->advertise<sensor_msgs::LaserScan>("laserscan_in_fuser_frame", 50);
        point2_publisher_ = nh_->advertise<sensor_msgs::PointCloud2>("point2_fuser", 15);
        pose_est_publisher_ = nh_->advertise<nav_msgs::Odometry>("fuser", 50);

        /*   if(gt_localisation)
          use_tf_listener_= use_tf_listener_ && state_base_link_id != std::string("");// check if
        odometry topic exists else use_tf_listener_= use_tf_listener_ && odometry_link_id !=
        std::string("");// check if odometry topic exists
    */

        if (!laser_2d)
        {
            velodyne_parser.setParameters(min_range, max_range, 0, 2 * M_PI);
            velodyne_parser.setupOffline(velodyne_config_file, max_range, min_range);
        }

        if (useOdometry) // start odometry early to get readings
            odom_sub_ =
                nh_->subscribe(odometry_topic, 100, &GraphLocalisationNode::OdometryCallback, this);

        cout << "Loading map at: " << map_path << endl;
        //FIXME: broken
        LoadGraphMap(map_path, map_path, graph_map_);

        // GraphPlot::PlotPoseGraph(graph_map_);
        // GraphPlot::PlotMap(graph_map_->GetCurrentMapNode(), -1,
        // graph_map_->GetCurrentNodePose(), point);
        ros::spinOnce();

        // GraphPlot::PlotPoseGraph(graph_map_);
        // GraphPlot::PlotMap(graph_map_->GetCurrentMapNode(), -1,
        // graph_map_->GetCurrentNodePose(), point);
        cout << "Initialization completed, starting localisation system" << endl;

        LocalisationParamPtr loc_ptr =
            LocalisationFactory::CreateLocalisationParam(localisation_type_name);

        loc_ptr->GetParamFromRos();
        loc_ptr->sensor_pose = Tsensor_;
        loc_ptr->graph_map_ = graph_map_;

        /*if(MCLNDTParamPtr parPtr=boost::dynamic_pointer_cast<MCLNDTParam>(loc_ptr ))
          GetMotionModel(dataset_name,parPtr->motion_model,parPtr->motion_model_offset);*/

        cout << "Localisation parameters:\n" << loc_ptr->ToString() << endl;
        localisation_type_ptr_ = LocalisationFactory::CreateLocalisationType(loc_ptr);

        if (!laser_2d) // init all callbacks
            velodyne_sub_ =
                nh_->subscribe(velodyne_topic, 1, &GraphLocalisationNode::PointCloudCallback, this);
        else
            laser_2d_sub_ =
                nh_->subscribe(laser_topic, 1, &GraphLocalisationNode::laserCallback, this);

        if (initPoseRos)
            init_pose_ros_sub_ =
                nh_->subscribe("/initialpose", 1, &GraphLocalisationNode::InitPoseRos, this);
        else if (initPoseGt)
            gt_state_sub_ = nh_->subscribe(gt_topic_, 1, &GraphLocalisationNode::GTCallback, this);
        else
        {
            // initial pose of the vehicle with respect to the map
            ///
            Eigen::Vector3d robot_init_pos, robot_init_euler;
            nh_->param("pose_init_x", robot_init_pos(0), 0.);
            nh_->param("pose_init_y", robot_init_pos(1), 0.);
            nh_->param("pose_init_z", robot_init_pos(2), 0.);
            nh_->param("pose_init_r", robot_init_euler(0), 0.);
            nh_->param("pose_init_p", robot_init_euler(1), 0.);
            nh_->param("pose_init_t", robot_init_euler(2), 0.);
            geometry_msgs::Pose p_tmp;
            tf::poseEigenToMsg(ndt_generic::vectorsToAffine3d(robot_init_pos, robot_init_euler),
                               p_tmp);
            Initialize(p_tmp, ros::Time::now());
            initPoseSet = true;
        }
        // GraphPlot::PlotMap()
    }

    void processFrame(pcl::PointCloud<pcl::PointXYZL>& cloud)
    {

        if (!initPoseSet)
            return;
        else
            frame_nr_++;
        ros::Time t_cloud;
        pcl_conversions::fromPCL(cloud.header.stamp, t_cloud);
        while (!odom_poses.GetPose(t_cloud.toSec(), odom_now))
            ;

        Eigen::Affine3d Tmotion = Odom_prev.inverse() * odom_now;

        cout << "frame nr=" << frame_nr_ << endl;
        if ((Tmotion.translation().norm() < 0.005 &&
             Tmotion.rotation().eulerAngles(0, 1, 2).norm() < 0.005))
        { // sanity check for odometry
            return;
        }
        ResetInvalidMotion(Tmotion);

        //  cout<<"frame="<<frame_nr_<<"movement="<<(fuser_->GetPoseLastFuse().inverse()*pose_).translation().norm()<<endl;
        pcl::PointCloud<pcl::PointXYZL> registered_cloud = cloud;
        ros::Time tplot = ros::Time::now();
        Eigen::MatrixXd cov(6, 6);
        cov = Eigen::MatrixXd::Identity(6, 6);

        //   m.lock();
        // bool uppdated=fuser_->ProcessFrame<pcl::PointXYZL>(cloud,pose_,Tmotion,cov);
        //   m.unlock();
        // fuser_->PlotMapType();
        tf::Transform Transform;
        tf::transformEigenToTF(pose_now, Transform);
        tf_.sendTransform(tf::StampedTransform(Transform, tplot, world_link_id, pose_est_link));
        tf::Transform Transform_sensor;
        tf::transformEigenToTF(pose_now * Tsensor_, Transform_sensor);
        tf_.sendTransform(tf::StampedTransform(Transform, tplot, pose_est_link, laser_link_id));
        fuser_odom.header.stamp = tplot;
        // plotPointcloud2(registered_cloud,tplot);
        tf::poseEigenToMsg(pose_now, fuser_odom.pose.pose);
        pose_est_publisher_.publish(fuser_odom);
    }

    // bool save_map_callback(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res )
    bool ResetInvalidMotion(Eigen::Affine3d& Tmotion)
    {
        if (Tmotion.translation().norm() > MAX_TRANSLATION_DELTA)
        {
            std::cerr << "Ignoring Odometry (max transl)!\n";
            std::cerr << Tmotion.translation().transpose() << std::endl;
            Tmotion.setIdentity();
            return true;
        }
        else if (Tmotion.rotation().eulerAngles(0, 1, 2)(2) > MAX_ROTATION_DELTA)
        {
            std::cerr << "Ignoring Odometry (max rot)!\n";
            std::cerr << Tmotion.rotation().eulerAngles(0, 1, 2).transpose() << std::endl;
            Tmotion.setIdentity();
            return true;
        }
        else
            return false;
    }

    // Callback
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
    {
        cout << "laser callback" << endl;
        /*  sensor_msgs::PointCloud2 cloud;
        pcl::PointCloud<pcl::PointXYZL> pcl_cloud_unfiltered, pcl_cloud;
        projector_.projectLaser(*msg_in, cloud);
        pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

        pcl::PointXYZL pt;
        //add some variance on z
        for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
          pt = pcl_cloud_unfiltered.points[i];
          if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_range) {
            pt.z += varz*((double)rand())/(double)INT_MAX;
            pcl_cloud.points.push_back(pt);
          }
        }
        T.setIdentity();
        cout<<"node: laser call back, process frame"<<endl;
        this->processFrame(pcl_cloud,T);
    */
    }

    void plotPointcloud2(pcl::PointCloud<pcl::PointXYZL>& cloud, ros::Time time)
    {
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(cloud, msg_out);
        msg_out.header.frame_id = laser_link_id;
        msg_out.header.stamp = time;
        point2_publisher_.publish(msg_out);
    }

    void OdometryCallback(const nav_msgs::Odometry::ConstPtr&
                              odom_in) // callback is used in conjunction with odometry time filter.
    {
        geometry_msgs::PoseStamped p_tmp;
        p_tmp.pose = odom_in->pose.pose;
        p_tmp.header.stamp = odom_in->header.stamp;
        // m_odom_poses.unlock();
        odom_poses.push(p_tmp);
        // m_odom_poses.lock();
        cout << "odometry callback" << endl;
    }

    void UnwrapCloudSimple(velodyne_rawdata::RawData& velodyneparser,
                           const velodyne_msgs::VelodyneScan::ConstPtr& msg_in,
                           pcl::PointCloud<pcl::PointXYZL>& cloud)
    {
        cloud.clear();
        pcl::PointCloud<pcl::PointXYZL> cloud_segment;
        velodyne_rawdata::VPointCloud pnts;
        for (int i = 0; i < msg_in->packets.size(); i++)
        {
            velodyneparser.unpack(msg_in->packets[i], pnts);
            pcl::copyPointCloud(pnts, cloud_segment);
            cloud += cloud_segment;
        }
        cloud.width = cloud.size();
        cloud.height = 1;
        ros::Time t_avg = msg_in->packets[0].stamp +
                          ros::Duration(msg_in->packets[msg_in->packets.size() - 1].stamp -
                                        msg_in->packets[0].stamp) *
                              0.5;
        pcl_conversions::toPCL(t_avg, cloud.header.stamp);
    }
    void PointCloudCallback(
        const velodyne_msgs::VelodyneScan::ConstPtr&
            msg_in) // callback is used in conjunction with odometry time filter.
    {
        if (!initPoseSet)
        {
            cout << "InitPose not set" << endl;
            return;
        }

        Eigen::Affine3d pose_tmp;
        //  cout<<"pose="<<pose_.translation().transpose()<<endl;
        pcl::PointCloud<pcl::PointXYZL> cloud;
        UnwrapCloudSimple(velodyne_parser, msg_in, cloud);
        processFrame(cloud);

        /*tf::Transform Transform;
          tf::transformEigenToTF(pose_tmp,Transform);
          tf_.sendTransform(tf::StampedTransform(Transform, t, world_link_id, pose_est_link));*/

        cout << "pose=" << pose_now.translation().transpose() << endl;
    }
    void Initialize(const geometry_msgs::Pose& pose_init, const ros::Time& t_init)
    {
        tf::poseMsgToEigen(pose_init, pose_now);
        t_now = t_init.toSec();
        cout << "Initial position set to" << pose_now.translation().transpose() << endl;
        Vector6d var;
        var << 1.5, 1.5, 0.0, 0.0, 0.0, 0.5;
        localisation_type_ptr_->InitializeLocalization(pose_now, var);
        pose_prev = pose_now;
        t_now = t_init.toSec();
        t_prev = t_now;
        initPoseSet = true;
    }

    void GTCallback(const nav_msgs::Odometry::ConstPtr&
                        state_msg_in) // callback is used in conjunction with odometry time filter.
    {
        if (!initPoseSet && !initPoseRos && initPoseGt)
        {
            Initialize(state_msg_in->pose.pose, state_msg_in->header.stamp);
            init_pose_ros_sub_.shutdown();
        }

        cout << "ground truth callback" << endl;
    }
    void InitPoseRos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_in)
    {
        if (!initPoseSet && initPoseRos)
        {
            Initialize(pose_in->pose.pose, pose_in->header.stamp);
            init_pose_ros_sub_.shutdown();
        }
        else
            cerr << "Pose already set" << endl;
    }

public:
    // map publishing function
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GraphLocalisationNode");
    ros::NodeHandle param("~");
    GraphLocalisationNode t(&param);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
