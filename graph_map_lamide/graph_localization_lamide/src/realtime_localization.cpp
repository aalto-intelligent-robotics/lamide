// perception_oru
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"

#include <message_filters/subscriber.h>

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
#include "boost/thread.hpp"
#include "graph_localization_lamide/localization_factory.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt.h"
#include "graph_localization_lamide/ukf_ndt/ukf_ndt.h"
#include "graph_localization_lamide/ukf_ndt/ukf_reg.h"
#include "graph_map_lamide/ndt_dl/point_curv3.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "ndt_generic_lamide/motion_model_2d.h"
#include "ndt_generic_lamide/motionmodels.h"
#include "ndt_generic_lamide/pcl_utils.h"
#include "std_srvs/Empty.h"
#include "std_srvs/TriggerRequest.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

using namespace perception_oru;
using namespace graph_map;
using namespace graph_localization;

class realtime_localization
{

private:
    ros::NodeHandle nh;
    // Map parameters
    std::string map_path, localization_type_name;
    GraphMapNavigatorPtr graph_map_;
    LocalisationTypePtr localisation_type_ptr_;
    std::string map_switching_method;
    // MCL
    bool visualize;
    graphVisualization* vis_;

    // laser input
    std::string points_topic; // std::string laserTopicName;

    ros::Publisher cloud_pub;
    ros::Publisher estPosePub;
    ros::Subscriber initPoseSub;
    ros::Subscriber PCSub;
    tf::Transform sensor_tf, robot_tf;

    Eigen::Affine3d Tsens;
    std::string root_link, base_link, mcl_link, gt_link, laser_link, pose_est_link;

    string localisation_type_name = "", registration_type_name = "";
    string dataset = "";
    int ms_sim_delay;
    int frame_rate;
    Eigen::Affine3d Tprev;
    bool firstLoad, gt_mockup;
    bool initialized, gt_initialize, map_localizaiton, use_ref_frame;

    double min_range;
    double max_range;
    Eigen::Affine3d pose_, pose_pred_;
    ros::Time t_latest, t_pred_latest;
    tf::TransformBroadcaster trans_pub;
    Eigen::Affine3d pose_init_offset_;
    ndt_generic::PointCloudQueue<pcl::PointXYZL> cloud_queue;
    ros::ServiceClient* client;
    boost::thread thread_output;
    mutex m_pose;

    int LoadMap(const std::string& path)
    {
        //FIXME: broken
        LoadGraphMap(path, path, graph_map_);

        if (graph_map_ == NULL)
        {
            std::cerr << "ERROR LOADING NDT MAP FROM FILE" << std::endl;
            exit(0);
        }
        if (use_ref_frame)
        {
            graph_map_->UpdatePoseFrame();
        }
    }

    void Initialize(const Eigen::Affine3d& pose_init, ros::Time t)
    {
        geometry_msgs::Pose pose_init_geom;
        tf::poseEigenToMsg(pose_init, pose_init_geom);
        Initialize(pose_init_geom, t);
    }

    void initialposeCallback(geometry_msgs::PoseWithCovarianceStamped input_init)
    {
        Initialize(input_init.pose.pose, input_init.header.stamp);
    }

    void Initialize(const geometry_msgs::Pose& pose_init, const ros::Time& t_init)
    {

        Eigen::Affine3d pose_init_eig;
        tf::poseMsgToEigen(pose_init, pose_init_eig);
        cout << "Initial position set to" << pose_init_eig.translation().transpose() << endl;
        Vector6d var;
        var << 0.5, 0.5, 0.0, 0.0, 0.0, 0.2;
        localisation_type_ptr_->InitializeLocalization(pose_init_eig, var);
        usleep(100);
        t_pred_latest = t_init;
        t_latest = t_init;
        initialized = true;
    }

    void LagCompensation()
    {
        tf::TransformListener tf_listener;
        tf::StampedTransform Tfodom, Tfodomprev;
        ros::Time todom;
        tf::Transform Tfmotion;
        Eigen::Affine3d Tmotion;
        ros::Rate loop_rate(frame_rate);
        while (ros::ok())
        {
            if (!initialized)
                continue; // if initiated
            m_pose.lock();

            try
            {
                tf_listener.lookupTransform(root_link, base_link, ros::Time(0), Tfodom);
                tf_listener.lookupTransform(root_link, base_link, t_latest, Tfodomprev);
                if (Tfodom.stamp_.toSec() < Tfodomprev.stamp_.toSec())
                {
                    m_pose.unlock();
                    continue;
                }
                Tfmotion = Tfodomprev.inverse() * Tfodom;
                tf::poseTFToEigen(Tfmotion, Tmotion);
                //   cout<<"Tmotion: "<<Tmotion.translation().transpose()<<endl;
                todom = Tfodom.stamp_;
            }
            catch (tf::TransformException ex)
            {
                cout << "no transf found" << endl;
                m_pose.unlock();
                continue;
            }
            if (todom.toSec() >= t_latest.toSec() && todom.toSec() > t_pred_latest.toSec() + 0.0001)
            { // prediction should be more recent than the cloud and previous prediction
                // cout<<"t since cloud: "<<todom.toSec()-t_latest.toSec()<<"since prediction:
                // "<<todom.toSec()-t_pred_latest.toSec()<<",
                // mot:"<<Tmotion.translation().transpose()<<endl;
                Eigen::Affine3d Tpred_eig = pose_ * Tmotion;
                PublishPose(Tpred_eig, todom);
                t_pred_latest = todom;
            }

            m_pose.unlock();
            loop_rate.sleep();
        }
    }
    void PublishPose(Eigen::Affine3d& T, ros::Time ts)
    {
        nav_msgs::Odometry odom_pose_est;
        odom_pose_est.header.frame_id = "/world";
        tf::poseEigenToMsg(T, odom_pose_est.pose.pose);
        odom_pose_est.header.stamp = ts;
        estPosePub.publish(odom_pose_est);
        tf::poseEigenToTF(T, robot_tf);
        trans_pub.sendTransform(tf::StampedTransform(robot_tf, ts, root_link, pose_est_link));
        tf::poseEigenToTF(Tsens, sensor_tf);
        trans_pub.sendTransform(tf::StampedTransform(sensor_tf, ts, pose_est_link, laser_link));
    }

    bool LookUpTransform(const std::string& lookup_link, ros::Time ts, Eigen::Affine3d& T)
    {
        static tf::TransformListener tf_listener;
        tf::StampedTransform transform;

        tf_listener.waitForTransform(root_link, lookup_link, ts, ros::Duration(0.1));
        try
        {
            tf_listener.lookupTransform(root_link, lookup_link, ts, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }
        tf::poseTFToEigen(transform, T);
        return true;
    }
    // template<typename PointT>
    /*bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL> &cloud, const Eigen::Affine3d &Tmotion,
       const Eigen::Affine3d &Tsensor)
       {
         return localisation_type_ptr_->UpdateAndPredict(cloud, Tmotion, Tsensor);
       }*/

    // template<>
    bool UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZL>> clouds;
        segmentPointCurvature3(Tsensor, cloud, clouds);
        // graph_map_->m_graph.lock();
        bool status = localisation_type_ptr_->UpdateAndPredict(clouds, Tmotion, Tsensor);
        // graph_map_->m_graph.unlock();
        return status;
    }

    // template <class PointT>
    void processFrame(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud, ros::Time ts)
    {

        static unsigned int frames = 0;
        static ros::Duration total = ros::Duration(0);

        Eigen::Affine3d T;
        if (!LookUpTransform(base_link, ts, T))
            return;

        Eigen::Affine3d Tinit;
        if (!initialized)
        { // Initialize map or dl localization
            if (gt_initialize)
            {
                if (!LookUpTransform(gt_link, ts, Tinit))
                    return;
            }
            else
                Tinit = Eigen::Affine3d::Identity();
            pose_ = Tinit;
            Initialize(Tinit, ts);
        }

        static Eigen::Affine3d Tprev = T;
        Eigen::Affine3d Tmotion = Tprev.inverse() * T;
        Tprev = T;

        transformPointCloudInPlace(Tsens, cloud);
        UpdateAndPredict(cloud, Tmotion, Tsens);
        frames++;
        m_pose.lock();
        pose_ = localisation_type_ptr_->GetPose();
        t_latest = ts;
        // PublishPose(pose_,t_latest);
        m_pose.unlock();
        cloud.clear();
    }
    void GtFeedback(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& pcl_cloud,
                    ros::Time tcloud)
    {
        cout << "GtFeedback" << endl;
        if (LookUpTransform(gt_link, tcloud, pose_))
        {
            if (ms_sim_delay > 0)
                usleep(1000 * ms_sim_delay);
            PublishPose(pose_, tcloud);
        }
    }
    void VelodyneCallback(
        const typename pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>::ConstPtr& msg)
    {

        ros::Time t = ros::Time::now();
        pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> pcl_cloud = *msg;
        ros::Time tcloud;
        pcl_conversions::fromPCL(msg->header.stamp, tcloud);

        if (!gt_mockup)
            this->processFrame(pcl_cloud, tcloud);
        else
            this->GtFeedback(pcl_cloud, tcloud);

        // cout<<"latency loalization: "<<ros::Time::now()-t<<endl;
    }

public:
    realtime_localization(ros::NodeHandle param) : initialized{false}, cloud_queue(10)
    {
        param.param<std::string>("map_file", map_path, "");
        param.param<bool>("visualize", visualize, true);
        param.param<bool>("gt_initialize", gt_initialize, true);
        param.param<bool>("map_localizaiton", map_localizaiton, true);
        param.param<bool>("use_ref_frame", use_ref_frame, true);
        param.param<bool>("gt_mockup", gt_mockup, true);

        param.param<std::string>("points_topic", points_topic, "/velodyne_points");

        param.param<std::string>("root_tf", root_link, "/world");
        param.param<std::string>("base_tf", base_link, "/robot_odom_link");
        param.param<std::string>("gt_tf", gt_link, "/state_base_link");
        param.param<std::string>("laser_tf", laser_link, "/velodyne");
        param.param<std::string>("pose_tf", pose_est_link, "/pose_est");
        param.param<std::string>("localization_type", localization_type_name, "ukf_reg");
        param.param<std::string>("registration_type_name", registration_type_name, "ndt_dl_reg");

        Eigen::Vector3d sensor_offset_pos, sensor_offset_euler;
        param.param("sensor_pose_x", sensor_offset_pos(0), 0.);
        param.param("sensor_pose_y", sensor_offset_pos(1), 0.);
        param.param("sensor_pose_z", sensor_offset_pos(2), 0.);
        param.param("sensor_pose_r", sensor_offset_euler(0), 0.);
        param.param("sensor_pose_p", sensor_offset_euler(1), 0.);
        param.param("sensor_pose_t", sensor_offset_euler(2), 0.);
        Tsens = ndt_generic::vectorsToAffine3d(sensor_offset_pos, sensor_offset_euler);

        Eigen::Matrix<double, 6, 1> init_vec;
        param.param("init_x", init_vec(0), 0.);
        param.param("init_y", init_vec(1), 0.);
        param.param("init_z", init_vec(2), 0.);
        param.param("init_ex", init_vec(0), 0.);
        param.param("init_ey", init_vec(1), 0.);
        param.param("init_ez", init_vec(2), 0.);
        param.param("ms_sim_delay", ms_sim_delay, 0);

        pose_init_offset_ = ndt_generic::xyzrpyToAffine3d(init_vec(0), init_vec(1), init_vec(2),
                                                          init_vec(3), init_vec(4), init_vec(5));
        pose_ = Eigen::Affine3d::Identity();

        param.param("min_range", min_range, 1.5);
        param.param("max_range", max_range, 130.0);
        param.param("max_range", max_range, 130.0);

        param.param<std::string>("dataset", dataset, "michigan");

        param.param<std::string>("map_switching_method", map_switching_method,
                                 "closest_observation");
        param.param<std::string>("localisation_type_name", localisation_type_name, "ukf_reg");

        LoadMap(map_path);

        LocalisationParamPtr loc_ptr =
            LocalisationFactory::CreateLocalisationParam(localisation_type_name);

        bool status = false;
        if (UKFRegParamPtr UKFparPtr = boost::dynamic_pointer_cast<UKFRegParam>(loc_ptr))
        {
            GetMotionModel(dataset, UKFparPtr->motion_model);
            if (graph_map::RegParamPtr reg_par_ptr =
                    GraphFactory::CreateRegParam(registration_type_name))
            {
                if (NDTDLRegParamPtr ndtdl_par_ptr =
                        boost::dynamic_pointer_cast<NDTDLRegParam>(reg_par_ptr))
                {
                    UKFparPtr->registration_parameters = ndtdl_par_ptr;
                    status = true;
                }
            }
        }
        if (!status)
        {
            std::cerr << "Error creating localizaiton types" << endl;
        }
        cout << "read ros" << endl;
        loc_ptr->GetParamFromRos();
        loc_ptr->switch_map_method =
            GraphMapNavigatorParam::String2SwitchMethod(map_switching_method);
        loc_ptr->sensor_pose = Tsens;
        loc_ptr->graph_map_ = graph_map_;
        localisation_type_ptr_ = LocalisationFactory::CreateLocalisationType(loc_ptr);

        cout << "-------------------------- Map and Localisation parameter "
                "----------------------------"
             << endl;
        cout << localisation_type_ptr_->ToString() << endl;
        cout << "--------------------------------------------------------" << endl;

        if (visualize)
            vis_ = new graphVisualization(graph_map_, true, true, true);
        PCSub = nh.subscribe<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>>(
            points_topic, 1, &realtime_localization::VelodyneCallback, this);

        // PCSub = nh.subscribe(points_topic, 1, &realtime_localization::PCCallback, this);
        cout << "Listen to sensor_msgs/PointCloud2 at topic \"" << points_topic << "\"" << endl;

        initPoseSub =
            nh.subscribe("/initialpose", 1000, &realtime_localization::initialposeCallback, this);
        cout << "Listen to initialization at \"/initialpose\"" << endl;

        estPosePub = nh.advertise<nav_msgs::Odometry>("pose_est", 20);
        cloud_pub = nh.advertise<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>>(
            "cloud_localized", 10);

        param.param("frame_rate", frame_rate, 14);
        cout << "boost framerate=" << frame_rate << endl;
        if (!gt_mockup)
        {
            thread_output = boost::thread(&realtime_localization::LagCompensation, this);
            // thread_output.join();
        }
        t_pred_latest = ros::Time(0);
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mcl");
    ros::NodeHandle parameters("~");
    bool use_pointtype_xyzir;
    parameters.param<bool>("use_pointtype_xyzir", use_pointtype_xyzir, "true");
    // if(use_pointtype_xyzir)
    realtime_localization pf(parameters);
    // else
    //   realtime_localization<pcl::PointXYZL> pf(parameters);
}
