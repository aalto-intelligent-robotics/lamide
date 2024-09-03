#pragma once

#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"
#include "ndt_generic_lamide/labels.h"
#include "ndt_localization_lamide/particle_filter.hpp"
#include "ndt_map_lamide/ndt_conversions.h"
#include "ndt_map_lamide/ndt_map.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// pcl
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
// ros
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/PoseArray.h"
#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/Odometry.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ndt_generic_lamide/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <velodyne_pointcloud_oru/rawdata.h>
// std
#include "graph_localization_lamide/localization_factory.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt_lamide.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "ndt_generic_lamide/io.h"
#include "ndt_generic_lamide/motion_model_2d.h"
#include "ndt_generic_lamide/motionmodels.h"
#include "ndt_generic_lamide/pcl_utils.h"
#include "ndt_generic_lamide/pcl_utils_impl.h"

#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <ctime>
#include <thread>
#include <mutex>
#include <queue>
#include <tgmath.h>

#include <std_srvs/Empty.h>

#include <csignal>
#include <cstdlib>

using namespace perception_oru;
using namespace graph_map;
using namespace graph_localization;

class localization_node
{
public:
    localization_node(ros::NodeHandle param);

    void histWriter();

    void fileWriter();

    void inputWatchdog();

    void requestStarter();

    void exitData();

    void exitRoutine();

private:
    void processFrame(pcl::PointCloud<pcl::PointXYZL>& cloud, const ros::Time& ts);

    Eigen::Affine3d getMotion(const ros::Time& ts, Eigen::Affine3d& Todom, bool& success);

    void filterPointsBySemantics(pcl::PointCloud<pcl::PointXYZL>& cloud) const;

    bool initLocalization(const ros::Time& ts);

    Eigen::Vector3d getError(const Eigen::Affine3d& pose,
                             const Eigen::Affine3d& gt,
                             double& error,
                             double& ATE,
                             double& RPE,
                             double& mean_rpe);

    Eigen::Affine3d getGT(const ros::Time& ts, ros::Time& gt_time);

    void printDebug(const Eigen::Affine3d& Tmotion,
                    const Eigen::Affine3d& pose,
                    const Eigen::Affine3d& gt,
                    const Eigen::Vector3d& error_vec,
                    double error,
                    double ATE,
                    double RPE,
                    double mean_rpe);

    void checkLost(double error);

    void publish(const ros::Time& ts,
                 const Eigen::Affine3d& pose,
                 pcl::PointCloud<pcl::PointXYZL>& cloud,
                 const Eigen::Affine3d& Todom);

    void log(const Eigen::Affine3d& pose,
             const Eigen::Affine3d& gt,
             const ros::Time& gt_time,
             const ros::Time& ts,
             const Eigen::Vector3d& error_vec,
             double ATE,
             double RPE,
             double mean_rpe) const;

    void histogramming(const pcl::PointCloud<pcl::PointXYZL>& cloud);

    void visualize(Eigen::Affine3d& pose, pcl::PointCloud<pcl::PointXYZL>& cloud);

    // INITIALIZE

    void Initialize(const Eigen::Affine3d& pose_init);

    void initialposeCallback(geometry_msgs::PoseWithCovarianceStamped input_init);

    void Initialize(const geometry_msgs::Pose& pose_init, const ros::Time& t_init);

    void InitializeUniform(const geometry_msgs::Pose& pose_init, const ros::Time& t_init);

    // CALLBACKS

    void VeloCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg);

    void PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    bool exit_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void GTCallback(const nav_msgs::Odometry::ConstPtr& msg_in);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_in);

    // UTILS

    inline bool file_exists(const std::string& name)
    {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    }

    bool logPose(std::string& filename, const Eigen::Affine3d& Ts, const ros::Time& ts) const;

    bool logDebug(std::string& filename,
                  const ros::Time& ts,
                  const Eigen::Vector3d& e,
                  const double& ate,
                  const double& rpe,
                  const double& rpe_mean,
                  const Eigen::Matrix3d& cov) const;

    bool logData(std::string& filename, const std::string& data) const;

    bool LoadAffineToFile(const std::string& fileName, Eigen::Affine3d& ret);

    void Pose2DToTF(Eigen::Vector3d mean, ros::Time ts, Eigen::Affine3d Todometry);

    int LoadMap();

    std::string getISOdate() const;

    void parseTransforms();

    void compare(const Eigen::Affine3d& pose);

    void requestScan();

    // ███╗   ███╗███████╗███╗   ███╗██████╗ ███████╗██████╗ ███████╗
    // ████╗ ████║██╔════╝████╗ ████║██╔══██╗██╔════╝██╔══██╗██╔════╝
    // ██╔████╔██║█████╗  ██╔████╔██║██████╔╝█████╗  ██████╔╝███████╗
    // ██║╚██╔╝██║██╔══╝  ██║╚██╔╝██║██╔══██╗██╔══╝  ██╔══██╗╚════██║
    // ██║ ╚═╝ ██║███████╗██║ ╚═╝ ██║██████╔╝███████╗██║  ██║███████║
    // ╚═╝     ╚═╝╚══════╝╚═╝     ╚═╝╚═════╝ ╚══════╝╚═╝  ╚═╝╚══════╝

    ros::NodeHandle nh;
    // Map parameters
    std::string map_path;
    std::string map_file;
    GraphMapNavigatorPtr graph_map_;
    GraphMapNavigatorPtr comp_map_;
    LocalisationTypePtr localisation_type_ptr_;
    // MCL
    bool doVisualization_;
    bool visualizationInitialized_ = false;
    int visualization_colors_ = 1;
    std::string initType;
    ros::Publisher mclPosePub;
    // laser input
    std::string input_cloud_topic, gt_topic; // std::string laserTopicName;

    ros::Publisher cloud_pub;
    ros::Subscriber initPoseSub;
    ros::Subscriber gtPoseSub;
    ros::Subscriber PCSub;
    tf::Transform Tcorr_tf, sensor_tf;

    Eigen::Affine3d Tsens;
    std::string rootTF;
    std::string odomTF;
    std::string baseTF;
    std::string mclTF;
    string localisation_type_name = "";
    string dataset = "";
    std::string initial_pose_topic;

    Eigen::Affine3d initial_pose;
    Eigen::Affine3d tOld;
    bool firstLoad_;
    geometry_msgs::PoseArray parMsg;
    double minx;
    double miny;

    double initVar;
    double var_x;
    double var_y;
    double var_th;
    double r_var_x;
    double r_var_y;
    double r_var_th;
    int tres;
    int counter;
    int counterLimit;
    int init_particles_ = 5000;

    // motion model
    double mm_tx1_;
    double mm_tx2_;
    double mm_ty1_;
    double mm_ty2_;
    double mm_tz1_;
    double mm_tz2_;
    double mm_rr1_;
    double mm_rr2_;
    double mm_rp1_;
    double mm_rp2_;
    double mm_ry1_;
    double mm_ry2_;
    double mm_off_tx_;
    double mm_off_ty_;
    double mm_off_tz_;
    double mm_off_rr_;
    double mm_off_rp_;
    double mm_off_ry_;

    // initialization distribution
    double init_range_x_;
    double init_range_y_;
    double init_range_z_;
    double init_range_rr_;
    double init_range_rp_;
    double init_range_ry_;

    std::ofstream res;

    std::string laser_link_id;
    std::string pose_estimate_topic;
    bool initialized_;
    double time_0;
    std::string undistorted_cloud_topic;

    velodyne_rawdata_oru::RawData dataParser;
    double min_range;
    double max_range;

    double v_size_x;
    double v_size_y;
    double v_size_z;
    double fraction;

    bool laser_2d;
    bool beVelodyne;
    bool bePC;
    bool init_pose_gt, init_pose_file;
    bool initial_pose_set = false;
    bool publish_sensor_link;
    tf::TransformBroadcaster trans_pub;
    graphVisualization* vis_;
    bool simple_vis;
    bool send_map_frame;
    bool verbose_;
    std::string worldTF;
    std::string pose_init_path;
    std::string log_prefix_;
    std::string hostname_;
    std::string sw_method_;
    double interchange_radius_;

    // use odom as is, or calc diff
    bool relative_movement_;

    // visualization
    int prev_node_id_ = -1;
    bool firstDraw_ = true;
    bool display_local_map_ = false;
    bool display_error_map_ = true;

    // note: own variables
    // histograms
    bool calculate_histogram_ = false;
    bool match_histogram_ = false;
    int histogram_ratio_ = -1;

    // localization lost
    double lost_threshold_ = -1;
    bool got_gt_ = false;
    int lost_count_ = 0;
    int error_dim_ = 3;
    int max_lost_ = 5;
    int frame_count_ = 0;

    // histogram
    bool input_timeout_ = false;
    bool input_time_set_ = false;
    std::chrono::time_point<std::chrono::steady_clock> last_input_time_;
    ros::ServiceServer exit_server_;

    std::string last_state_print_ = "";
    std::string lost_print_ = "";
    std::string exitPath_;
    std::string exitMetadataPath_;

    // note: lamide params
    bool distribution_error_ = false;
    bool restrict_to_2d_solution_ = false;
    bool odom_twist_ = false;
    std::string odom_type_ = "";
    Eigen::Affine3d latest_odom_;
    bool got_odom_ = false;
    long double error_sum_ = 0;
    std::deque<nav_msgs::Odometry> gt_buffer_;
    int gt_buffer_max_size_ = 10;
    Eigen::Affine3d last_gt_;
    Eigen::Affine3d last_pose_;
    bool got_last_ = false;
    long double rpe_sum_ = 0;
    double estimated_rate_ = 0;
    std::deque<double> rateMeasurements_;
    bool isLifelong_ = false;

    bool compare_ = false;
    std::string comp_map_file_ = "";
    std::string comp_map_path_ = "";
    std::string transforms_file_ = "";
    std::vector<Eigen::Affine3d> tfs_;

    bool request_scans_ = false;
    bool request_got_next_ = true;
    bool got_frame_ = false;
    std::string scan_service_ = "/oxford_publisher/publish";

    struct HistogramObject
    {
        HistogramObject(){};
        HistogramObject(const pcl::PointCloud<pcl::PointXYZL>& cloud,
                        const Eigen::Affine3d& pose,
                        int id)
            : cloud_(cloud)
            , pose_(pose)
            , node_id_(id)
        {
        }
        pcl::PointCloud<pcl::PointXYZL> cloud_;
        Eigen::Affine3d pose_;
        int node_id_;
    };

    std::queue<HistogramObject> histogram_buffer_;
    // std::mutex histogram_mutex_;
    // std::mutex histogram_data_mutex_;
    std::string histogramPath_;
    std::vector<std::string> histogram_data_;
    bool map_unloading_;
    bool only_static_;
    bool filter_dynamic_;
};