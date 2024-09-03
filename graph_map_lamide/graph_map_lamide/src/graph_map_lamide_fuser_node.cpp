// Workaround for lz4 conflicting declaration -----------------------------//
// see https://github.com/ethz-asl/lidar_align/issues/16#issuecomment-758960226
#define LZ4_stream_t                        LZ4_stream_t_deprecated
#define LZ4_resetStream                     LZ4_resetStream_deprecated
#define LZ4_createStream                    LZ4_createStream_deprecated
#define LZ4_freeStream                      LZ4_freeStream_deprecated
#define LZ4_loadDict                        LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue          LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict                        LZ4_saveDict_deprecated
#define LZ4_streamDecode_t                  LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue               LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode              LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode                LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode                 LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue        LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue        LZ4_decompress_fast_continue_deprecated
#include <rosbag/bag.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue
//-------------------------------------------------------------------------//

#include "graph_map_lamide/graph_map_fuser.h"

#include <ros/ros.h>
// #include <rosbag/bag.h>
#include "eigen_conversions/eigen_msg.h"
#include "graph_map_lamide/lidarUtils/lidar_utilities.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "message_filters/subscriber.h"
#include "ndt_generic_lamide/pcl_utils.h"
#include "ndt_generic_lamide/utils.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"

#include <Eigen/Eigen>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
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
#include <pcl_ros/transforms.h>
#include <rosbag/view.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <time.h>
#include <visualization_msgs/MarkerArray.h>

#ifndef SYNC_FRAMES
#define SYNC_FRAMES           20
#define MAX_TRANSLATION_DELTA 2.0
#define MAX_ROTATION_DELTA    0.5
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */
namespace po = perception_oru;
namespace pogm = perception_oru::graph_map;
using std::cerr;
using std::cout;
using std::endl;
using std::string;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        nav_msgs::Odometry>
    PointsOdomSync;

bool save_occupancy;

class GraphMapFuserNode
{
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;
    pogm::GraphMapFuser* fuser_ = NULL;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* points2_sub_;
    ros::Subscriber laser_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_;
    ros::Subscriber pointcloud_callback, sensor_callback;
    pogm::plotmarker plot_marker;
    bool use_pointcloud;
    message_filters::Subscriber<nav_msgs::Odometry>* gt_fuser_sub_;
    ros::Subscriber gt_sub;

    // Components for publishing
    tf::TransformBroadcaster tf_;
    tf::TransformListener tf_listener_;
    ros::Publisher output_pub_;
    Eigen::Affine3d pose_;
    Eigen::Affine3d T;
    Eigen::Affine3d sensorPose_;
    Eigen::Affine3d previousPose_;
    Eigen::Affine3d initialGuess_;
    tf::StampedTransform T_odom;
    bool first_ = true;

    unsigned int frame_nr_;
    double varz;
    tf::Transform tf_sensor_pose_;
    std::string map_type_name;
    std::string reg_type_name;
    std::string map_name = "graph_map";
    std::string points_topic = "";
    std::string map_dir;
    std::string odometry_topic;
    std::string odometry_adjusted_topic;
    std::string file_format_map = ".JFF";
    std::string world_link_id;
    std::string fuser_base_link_id;
    std::string laser_link_id;
    std::string init_pose_frame;
    std::string gt_topic;
    std::string bag_name;
    std::string state_base_link_id;
    std::string map_prefix_;
    double size_x;
    double size_y;
    double size_z;
    double resolution;
    double max_range;
    double min_range;
    bool visualize;
    bool match2D;
    bool laser_2d;
    bool beHMT;
    bool useOdometry;
    bool initPoseFromGT;
    bool initPoseFromOdom;
    bool initPoseFromTF;
    bool initPoseSet;
    bool gt_mapping;
    bool use_odom_as_gt_;
    bool relative_movement_;
    double pose_init_x;
    double pose_init_y;
    double pose_init_z;
    double pose_init_r;
    double pose_init_p;
    double pose_init_t;
    double sensor_pose_x;
    double sensor_pose_y;
    double sensor_pose_z;
    double sensor_pose_r;
    double sensor_pose_p;
    double sensor_pose_t;
    double sensor_offset_t_;
    std::string output_pointcloud_topic_name;
    bool do_soft_constraints;
    std::string occupancy_topic;
    std::string sensor_pose_topic;
    laser_geometry::LaserProjection projector_;
    message_filters::Synchronizer<PointsOdomSync>* sync_po_;

    ros::ServiceServer save_map_;
    ros::ServiceServer exit_server_;
    ros::Time time_now;
    ros::Time time_last_itr;
    ros::Publisher map_publisher_;
    ros::Publisher laser_publisher_;
    ros::Publisher point2_publisher_;
    ros::Publisher odom_publisher_;
    ros::Publisher adjusted_odom_publisher_;
    ros::Publisher fuser_odom_publisher_;
    nav_msgs::Odometry fuser_odom;
    nav_msgs::Odometry adjusted_odom_msg;
    Eigen::Affine3d last_gt_pose;
    pogm::graphVisualization* vis;
    int color_;
    bool use_tf_listener_;
    bool use_REP_convention;
    bool publish_laser_link;
    Eigen::Affine3d last_tf_frame_;
    perception_oru::MotionModel2d::Params motion_params;
    boost::mutex m;
    pogm::GraphMapNavigatorPtr graph_map;
    std::string odometry_link_id;
    std::string base_link_id;
    bool ndt_om_ = false;
    bool only_static_ = false;
    bool clusterUpdate_ = false;
    bool request_scans_ = false;
    bool request_got_next_ = false;
    std::string scan_service_ = "/kitti_publisher/publish";

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //  ██████╗████████╗ ██████╗ ██████╗
    // ██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗
    // ██║        ██║   ██║   ██║██████╔╝
    // ██║        ██║   ██║   ██║██╔══██╗
    // ╚██████╗   ██║   ╚██████╔╝██║  ██║
    //  ╚═════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝

    // Constructor
    GraphMapFuserNode(ros::NodeHandle param_nh)
        : frame_nr_(0)
    {
        ROS_INFO("THIS IS LAMIDE NDT FUSER");

        /* ------------------------------- local vars ------------------------------- */
        bool do_soft_constraints;
        std::string marker_str;

        /* -------------------------------------------------------------------------- */
        /*                                 read params                                */
        /* -------------------------------------------------------------------------- */

        // topic to wait for point clouds, if available
        param_nh.param<std::string>("points_topic", points_topic, "");

        // enable for LaserScan message input
        param_nh.param("laser_2d", laser_2d, true);

        // Min and max range  for sensor measurements filtering
        param_nh.param<double>("max_range", max_range, 30.0);
        param_nh.param("min_range", min_range, 0.1);

        // visualize in a local window
        param_nh.param("visualize", visualize, true);

        param_nh.param<std::string>("plot_marker", marker_str, "point");
        plot_marker = pogm::plotmarker::sphere;

        // if using the HMT fuser, NDT maps are saved in this directory.
        // a word of warning: if you run multiple times with the same directory,
        // the old maps are loaded automatically
        param_nh.param<std::string>("map_directory", map_dir, "/maps/");

        param_nh.param<std::string>("occupancy_topic", occupancy_topic, "/projected_map");

        param_nh.param<bool>("save_occupancy", save_occupancy, true);

        param_nh.param<std::string>("map_type", map_type_name, "ndt_map");
        param_nh.param<std::string>("registration_type", reg_type_name, "default_reg");

        param_nh.param<std::string>("file_format_map", file_format_map, ".map");

        // initial pose of the vehicle with respect to the map
        param_nh.param("pose_init_x", pose_init_x, 0.);
        param_nh.param("pose_init_y", pose_init_y, 0.);
        param_nh.param("pose_init_z", pose_init_z, 0.);
        param_nh.param("pose_init_r", pose_init_r, 0.);
        param_nh.param("pose_init_p", pose_init_p, 0.);
        param_nh.param("pose_init_t", pose_init_t, 0.);

        // pose of the sensor with respect to the vehicle odometry frame
        param_nh.param("sensor_pose_x", sensor_pose_x, 0.);
        param_nh.param("sensor_pose_y", sensor_pose_y, 0.);
        param_nh.param("sensor_pose_z", sensor_pose_z, 0.);
        param_nh.param("sensor_pose_r", sensor_pose_r, 0.);
        param_nh.param("sensor_pose_p", sensor_pose_p, 0.);
        param_nh.param("sensor_pose_t", sensor_pose_t, 0.);
        param_nh.param("sensor_offset_t", sensor_offset_t_, 0.);
        // size of the map in x/y/z. if using HMT, this is the size of the central tile
        param_nh.param("size_x_meters", size_x, 10.);
        param_nh.param("size_y_meters", size_y, 10.);
        param_nh.param("size_z_meters", size_z, 10.);

        param_nh.param<double>("motion_params_Cd", motion_params.Cd, 0.005);
        param_nh.param<double>("motion_params_Ct", motion_params.Ct, 0.01);
        param_nh.param<double>("motion_params_Dd", motion_params.Dd, 0.001);
        param_nh.param<double>("motion_params_Dt", motion_params.Dt, 0.01);
        param_nh.param<double>("motion_params_Td", motion_params.Td, 0.001);
        param_nh.param<double>("motion_params_Tt", motion_params.Tt, 0.005);

        param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
        param_nh.param("laser_variance_z", varz, 0.1);

        // if we want to create map based on GT pose
        param_nh.param("renderGTmap", gt_mapping, false);
        param_nh.param<std::string>("gt_topic", gt_topic, "groundtruth");
        param_nh.param<bool>("use_pointcloud", use_pointcloud, false);

        // if we want to get the initial pose of the vehicle relative to a different frame
        param_nh.param("initPoseFromGT", initPoseFromGT, false);
        param_nh.param("initPoseFromOdom", initPoseFromOdom, false);
        // plot the map from the GT track if available

        // topic to wait for laser scan messages, if available
        param_nh.param<std::string>("odometry_topic", odometry_topic, "odometry");

        param_nh.param("initPoseFromTF", initPoseFromTF, false);

        // the frame to initialize to
        param_nh.param<std::string>("world_frame", world_link_id, "/world");
        // our frame
        param_nh.param<std::string>("laser_frame_id", laser_link_id,
                                    "/laser_link"); // The link of the output point cloud

        param_nh.param<std::string>("state_base_link_id", state_base_link_id, "/state_base_link");
        param_nh.param<std::string>("output_pointcloud_topic_name", output_pointcloud_topic_name,
                                    "/fused_cloud");

        // use standard odometry messages for initialuess
        param_nh.param("use_odometry_message", useOdometry, false);

        param_nh.param<bool>("use_tf_listener", use_tf_listener_, false);

        param_nh.param<std::string>("odom_tf", odometry_link_id, std::string("/odom_base_link"));
        param_nh.param<std::string>("base_tf", base_link_id, std::string("/base_footprint"));
        param_nh.param<std::string>("sensor_offset_topic", sensor_pose_topic,
                                    std::string("/sensor_default"));
        param_nh.param<bool>("use_REP_convention", use_REP_convention, false);
        param_nh.param<bool>("publish_laser_link", publish_laser_link, true);

        // note custom params
        param_nh.param<bool>("ndt_om", ndt_om_, false);
        param_nh.param<bool>("only_static", only_static_, false);

        param_nh.param<bool>("use_odom_as_gt", use_odom_as_gt_, false);
        param_nh.param<bool>("relative_movement", relative_movement_, false);

        param_nh.param<std::string>("map_prefix", map_prefix_, "");
        param_nh.param<int>("color", color_, 1);

        param_nh.param<bool>("cluster_update", clusterUpdate_, false);

        param_nh.param<bool>("request_scans", request_scans_, false);
        param_nh.param<std::string>("scan_service", scan_service_, "/kitti_publisher/publish");
        double presleep;
        param_nh.param<double>("presleep", presleep, 5.0);

        /* -------------------------------------------------------------------------- */
        /*                             publish & subscribe                            */
        /* -------------------------------------------------------------------------- */
        fuser_base_link_id = base_link_id;
        initPoseSet = false;
        fuser_odom.header.frame_id = world_link_id;

        point2_publisher_ =
            param_nh.advertise<sensor_msgs::PointCloud2>(output_pointcloud_topic_name, 15);
        fuser_odom_publisher_ = param_nh.advertise<nav_msgs::Odometry>("ndt_pose_est", 50);

        sensorPose_ = Eigen::Translation<double, 3>(sensor_pose_x, sensor_pose_y, sensor_pose_z) *
                      Eigen::AngleAxis<double>(sensor_pose_r, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxis<double>(sensor_pose_p, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxis<double>(sensor_pose_t, Eigen::Vector3d::UnitZ());

        cout << "sens:" << sensorPose_.translation().transpose() << endl;
        tf::poseEigenToTF(sensorPose_, tf_sensor_pose_);

        sensor_callback = nh_.subscribe<geometry_msgs::Pose>(
            sensor_pose_topic, 1, &GraphMapFuserNode::SensorOffsetCallback, this);

        /* ---------------------------- laser & odometry ---------------------------- */

        if (laser_2d)
        {
            laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
                points_topic, 10, &GraphMapFuserNode::laserCallback, this);
            if (use_tf_listener_)
                cout << "Using 2d laser with TF listener at topic =" << points_topic << endl;
            else
                cout << "using 2d laser without odometry at topic: " << points_topic << endl;
        }
        else if (useOdometry)
        {
            use_tf_listener_ = false;
            cout << "Using synchronized odometry and velodyne messages. Consider using TF's "
                    "instead of this"
                 << endl;
            odom_sub_ =
                new message_filters::Subscriber<nav_msgs::Odometry>(nh_, odometry_topic, 10);
            sync_po_ = new message_filters::Synchronizer<PointsOdomSync>(
                PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
            sync_po_->registerCallback(
                boost::bind(&GraphMapFuserNode::SyncedPointsOdomCallback, this, _1, _2));
        }
        else if (use_tf_listener_)
        {
            pointcloud_callback = nh_.subscribe<sensor_msgs::PointCloud2>(
                points_topic, 10, &GraphMapFuserNode::pointcloudCallback, this);
        }
        else
        {
            cout << "Subscribe (without odometry ) to sensor_msgs::PointCloud2 at topic: "
                 << points_topic << endl;
            pointcloud_callback = nh_.subscribe<sensor_msgs::PointCloud2>(
                points_topic, 10, &GraphMapFuserNode::pointcloudCallback, this);
        }

        if (points_topic == "")
        {
            cerr << "No topic specified" << endl;
        }
        else
        {
            cout << "init done... waiting for data on topic\"" << points_topic << endl;
            if (useOdometry && !use_tf_listener_)
            {
                cout << "Expecting odometry at topic: " << odometry_topic << endl;
            }
            else if (!useOdometry && use_tf_listener_)
            {
                cout << "Expecting TF at " << odometry_link_id << " and " << base_link_id << endl;
            }
        }

        /* ------------------------------ ground truth ------------------------------ */

        if (initPoseFromGT)
        {
            cout << "Init pose using GT data from topic =" << gt_topic << endl;
            gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic, 10,
                                                       &GraphMapFuserNode::gt_callback, this);
        }
        else if (!initPoseFromGT and !initPoseFromOdom)
        {
            pose_ = Eigen::Translation<double, 3>(pose_init_x, pose_init_y, pose_init_z) *
                    Eigen::AngleAxis<double>(pose_init_r, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxis<double>(pose_init_p, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxis<double>(pose_init_t, Eigen::Vector3d::UnitZ());
            initPoseSet = true;
            cout << "node: initial pose =\n" << pose_.translation() << endl;
        }

        save_map_ =
            param_nh.advertiseService("save_map", &GraphMapFuserNode::save_map_callback, this);
        exit_server_ = param_nh.advertiseService("exit", &GraphMapFuserNode::exit_callback, this);
        po::NDTCell::setParameters(0.1, 8 * M_PI / 18., 1000, 3, true, false);

        ROS_INFO("Ctor done.");

        if(request_scans_)
        {
            long ms = 1000 * presleep;
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            requestScan();
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            requestScan();
        }
    }

    void requestScan()
    {
        std_srvs::Trigger service;
        ros::service::call(scan_service_, service);
        request_got_next_ = service.response.success;
    }

    // ███╗   ██╗ ██████╗      ██████╗ ██████╗  ██████╗ ███╗   ███╗
    // ████╗  ██║██╔═══██╗    ██╔═══██╗██╔══██╗██╔═══██╗████╗ ████║
    // ██╔██╗ ██║██║   ██║    ██║   ██║██║  ██║██║   ██║██╔████╔██║
    // ██║╚██╗██║██║   ██║    ██║   ██║██║  ██║██║   ██║██║╚██╔╝██║
    // ██║ ╚████║╚██████╔╝    ╚██████╔╝██████╔╝╚██████╔╝██║ ╚═╝ ██║
    // ╚═╝  ╚═══╝ ╚═════╝      ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝     ╚═╝

    Eigen::Affine3d getInitialGuess()
    {
        if (first_)
        {
            return Eigen::Affine3d::Identity();
        }
        else
        {
            return initialGuess_;
        }
    }

    void saveInitialGuess()
    {
        if (first_)
        {
            first_ = false;
        }
        else
        {
            initialGuess_ = previousPose_ * pose_.inverse();
        }

        previousPose_ = pose_;
    }

    // ███████╗██████╗  █████╗ ███╗   ███╗███████╗
    // ██╔════╝██╔══██╗██╔══██╗████╗ ████║██╔════╝
    // █████╗  ██████╔╝███████║██╔████╔██║█████╗
    // ██╔══╝  ██╔══██╗██╔══██║██║╚██╔╝██║██╔══╝
    // ██║     ██║  ██║██║  ██║██║ ╚═╝ ██║███████╗
    // ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝

    void processFrame(pcl::PointCloud<pcl::PointXYZL>& cloud, Eigen::Affine3d Tmotion)
    {
        if (!initPoseSet)
        {
            if (initPoseFromOdom)
            {
                ros::Time ts = ros::Time(0);
                Eigen::Affine3d Todo_eig;
                if (!GetTransformFromTF(ts, T_odom))
                {
                    return;
                }
                tf::poseTFToEigen(T_odom, Todo_eig);
                pose_ = Todo_eig;
                initPoseSet = true;
                std::cout << "Init pose set!" << std::endl;
                return;
            }
            else
            {
                std::cout << "no init pose set" << std::endl;
                return;
            }
        }

        //
        // ─── INITIAL GUESS ───────────────────────────────────────────────
        //

        // TODO: this was required by oxford as well
        ros::Time ts = ros::Time(0);
        //  This is the initial guess
        Eigen::Affine3d Tm = Tmotion;

        // Tf publishing
        if (use_tf_listener_)
        {
            Eigen::Affine3d Todo_eig;
            if (!GetTransformFromTF(ts, T_odom))
            {
                return;
            }
            tf::poseTFToEigen(T_odom, Todo_eig);

            //! this controls the input odometry
            if (relative_movement_)
            {
                Tm = Todo_eig;
            }
            else
            {
                Tm = GetOdomdiff(Todo_eig);
            }
        }
        // This is no odometry, no tf -> no initial guess
        else if (!useOdometry)
        {
            Tm = getInitialGuess();
        }

        //
        // ─── INITIALIZE ──────────────────────────────────────────────────
        //

        // Initialize fuser
        if (fuser_ == NULL)
        {
            fuser_ = new pogm::GraphMapFuser(map_type_name, reg_type_name, pose_ * sensorPose_,
                                             sensorPose_, map_dir, ndt_om_, only_static_);
            cout << fuser_->ToString() << endl;
            graph_map = fuser_->GetGraph();
            if (visualize)
            {
                vis = new pogm::graphVisualization(graph_map, true, color_);
                vis->SetMarkerType(plot_marker);
                vis->SetParentFrameId(world_link_id);
            }
            // this is need for fusing without odometry
            fuser_->SetkeyframeOptions(true);
            fuser_->setUseOdomAsGT(use_odom_as_gt_);
            fuser_->setClusterUpdate(clusterUpdate_);
        }

        //
        // ─── FUSE ────────────────────────────────────────────────────────
        //

        // Fuse frame
        pcl::PointCloud<pcl::PointXYZL> registered_cloud = cloud;
        po::transformPointCloudInPlace(sensorPose_, cloud);

        bool updated = fuser_->ProcessFrame<pcl::PointXYZL>(cloud, pose_, Tm);

        //
        // ─── PUBLISH TRANSFORMS ──────────────────────────────────────────
        //

        // Publish sensor link
        if (publish_laser_link && use_REP_convention)
        {
            tf_.sendTransform(
                tf::StampedTransform(tf_sensor_pose_, ts, base_link_id, laser_link_id));
        }
        else if (publish_laser_link && !use_REP_convention)
        {
            tf_.sendTransform(
                tf::StampedTransform(tf_sensor_pose_, ts, fuser_base_link_id, laser_link_id));
        }

        // Publish pose
        tf::Transform Transform;
        tf::transformEigenToTF(pose_, Transform);

        // publish correction transformations
        if (use_REP_convention)
        {
            tf::Transform tf_pose;
            tf::poseEigenToTF(pose_, tf_pose);
            tf::Transform Tcorr_tf = tf_pose * T_odom.inverse();
            tf_.sendTransform(tf::StampedTransform(Tcorr_tf, ts, world_link_id, odometry_link_id));
        }
        // publish in a few branch
        else
        {
            tf_.sendTransform(
                tf::StampedTransform(Transform, ts, world_link_id, fuser_base_link_id));
        }
        if (updated)
        {
            fuser_odom.header.stamp = ts;
            tf::poseEigenToMsg(pose_, fuser_odom.pose.pose);
            fuser_odom_publisher_.publish(fuser_odom);
            plotPointcloud2(registered_cloud, ts);

            frame_nr_++;
        }

        saveInitialGuess();

        if (request_scans_ && request_got_next_)
        {
            requestScan();
        }
    }

    // ██╗   ██╗████████╗██╗██╗     ███████╗
    // ██║   ██║╚══██╔══╝██║██║     ██╔════╝
    // ██║   ██║   ██║   ██║██║     ███████╗
    // ██║   ██║   ██║   ██║██║     ╚════██║
    // ╚██████╔╝   ██║   ██║███████╗███████║
    //  ╚═════╝    ╚═╝   ╚═╝╚══════╝╚══════╝

    void plotPointcloud2(pcl::PointCloud<pcl::PointXYZL>& cloud, ros::Time time = ros::Time::now())
    {
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(cloud, msg_out);
        msg_out.header.frame_id = laser_link_id;
        msg_out.header.stamp = time;
        point2_publisher_.publish(msg_out);
    }

    bool save_map_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        char path[1000];
        string time = ndt_generic::currentDateTimeString();

        if (fuser_ != NULL)
        {
            if (file_format_map.compare(".JFF") == 0)
            {
                fuser_->SaveCurrentNodeAsJFF(map_dir + "/" + "ndt_map.JFF");
            }
            else
            {
                fuser_->SaveGraphMap(map_dir, "ndt_map.MAP", map_prefix_);
            }
            std::cout << "Done!" << std::endl;
            return true;
        }
        else
            ROS_INFO("No data to save");
        return false;
    }

    bool exit_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        std::cout << "ros shutdown" << std::endl;
        ros::shutdown();
        return true;
    }

    void SensorOffsetCallback(const geometry_msgs::PoseConstPtr& msg)
    {
        tf::poseMsgToTF(*msg, tf_sensor_pose_);
        ROS_INFO("changed sensor offset wrt. odometry link");
    }

    // ████████╗███████╗
    // ╚══██╔══╝██╔════╝
    //    ██║   █████╗
    //    ██║   ██╔══╝
    //    ██║   ██║
    //    ╚═╝   ╚═╝

    bool GetTransformFromTF(const ros::Time& time, tf::StampedTransform& transform)
    {
        static tf::TransformListener tf_listener;

        // TODO: the duration was required by Oxford data, maybe add switch?
        tf_listener.waitForTransform(odometry_link_id, base_link_id, time, ros::Duration(10));
        try
        {
            tf_listener.lookupTransform(odometry_link_id, base_link_id, time, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            cerr << "No transformation between " << odometry_link_id << ", and " << base_link_id
                 << " at time" << time.toSec() << endl;
            return false;
        }
        return true;
    }

    Eigen::Affine3d GetOdomdiff(const geometry_msgs::Pose& odom_msg)
    {
        Eigen::Affine3d T;
        tf::poseMsgToEigen(odom_msg, T);
        return GetOdomdiff(T);
    }

    Eigen::Affine3d GetOdomdiff(const Eigen::Affine3d& T)
    {
        static Eigen::Affine3d last_odom = T;
        Eigen::Affine3d Tm = last_odom.inverse() * T;
        last_odom = T;
        return Tm;
    }

    // callback is used in conjunction with odometry time filter.
    void SyncedPointsOdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                                  const nav_msgs::Odometry::ConstPtr& odo_in)
    {
        pcl::PointCloud<pcl::PointXYZL> cloud;
        Eigen::Affine3d Tm = GetOdomdiff(odo_in->pose.pose);
        this->processFrame(cloud, Tm);
    }

    // ██╗      █████╗ ███████╗███████╗██████╗
    // ██║     ██╔══██╗██╔════╝██╔════╝██╔══██╗
    // ██║     ███████║███████╗█████╗  ██████╔╝
    // ██║     ██╔══██║╚════██║██╔══╝  ██╔══██╗
    // ███████╗██║  ██║███████║███████╗██║  ██║
    // ╚══════╝╚═╝  ╚═╝╚══════╝╚══════╝╚═╝  ╚═╝

    // 2D Callback
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
    {
        sensor_msgs::PointCloud2 cloud;
        pcl::PointCloud<pcl::PointXYZL> pcl_cloud_unfiltered, pcl_cloud;
        projector_.projectLaser(*msg_in, cloud);

        pcl::fromROSMsg(cloud, pcl_cloud_unfiltered);

        pcl::PointXYZL pt;
        // add some variance on z
        for (int i = 0; i < pcl_cloud_unfiltered.points.size(); i++)
        {
            pt = pcl_cloud_unfiltered.points[i];
            if (sqrt(pt.x * pt.x + pt.y * pt.y) > min_range)
            {
                pt.z += varz * ((double)rand()) / (double)INT_MAX;
                pcl_cloud.points.push_back(pt);
            }
        }
        pcl_conversions::toPCL(msg_in->header.stamp, pcl_cloud.header.stamp);

        Eigen::Affine3d Tm = Eigen::Affine3d::Identity();
        this->processFrame(pcl_cloud, Tm);
    }

    // 3D pointcloud
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
    {
        pcl::PointCloud<pcl::PointXYZL> cloud;
        pcl::fromROSMsg(*msg_in, cloud);
        pcl_conversions::toPCL(msg_in->header.stamp, cloud.header.stamp);

        // BUG: this affects something else than filtering!
        if (min_range > 0.2)
        {
            int numpts = cloud.points.size();
            int cnt = 0;
            std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>> filtered;
            filtered.resize(numpts);
            for (int i = 0; i < numpts; i++)
            {
                pcl::PointXYZL pt = cloud.points[i];
                if (sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > min_range)
                {
                    filtered[cnt] = pt;
                    cnt++;
                }
            }
            cloud.points.swap(filtered);
        }

        Eigen::Affine3d Tm = Eigen::Affine3d::Identity();
        this->processFrame(cloud, Tm);
    }

    // This callback is used to set initial pose from GT data.
    void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
    {
        if (initPoseFromGT && !initPoseSet)
        {
            Eigen::Affine3d gt_pose;
            tf::poseMsgToEigen(msg_in->pose.pose, gt_pose);
            pose_ = gt_pose;
            initPoseSet = true;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_map_lamide_fuser_node");
    ros::NodeHandle param("~");
    GraphMapFuserNode t(param);
    ros::spin();

    return 0;
}
