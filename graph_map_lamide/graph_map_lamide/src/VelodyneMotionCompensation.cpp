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
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <ndt_generic_lamide/eigen_utils.h>
#include <ndt_generic_lamide/pcl_utils.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ndt_generic_lamide/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <velodyne_pointcloud_oru/rawdata.h>
// std
#include <eigen_conversions/eigen_msg.h>
#include <ndt_generic_lamide/pcl_utils.h>
#include <ndt_map_lamide/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <string>


using std::cout;
using std::endl;
class localisation_node
{
    ros::NodeHandle nh;
    // Map parameters
    // MCL

    ros::Publisher cloud_pub;
    ros::Subscriber cloud_sub;
    ros::Subscriber sensor_offset_sub;

    std::string input_points_topic, output_points_topic, sensor_offset_topic;
    std::string odom_link, odom_child_link, output_points_link;
    tf::TransformBroadcaster trans_pub;
    velodyne_rawdata_oru::RawData dataParser;
    tf::TransformListener tf_listener;
    double min_range, max_range;
    Eigen::Affine3d Tsensor;
    bool disable_compensation;

    void SensorOffsetCallback(const geometry_msgs::PoseConstPtr& msg)
    {
        // tf::poseMsgToTF(*msg, Tsensor);:
        tf::poseMsgToEigen(*msg, Tsensor);
        ROS_INFO("changed sensor offset wrt. odometry link");
    }
    template <class PointT> void VeloCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg)
    {

        velodyne_msgs::VelodyneScan::ConstPtr scan = msg;
        // ros::Time t0 = ros::Time::now();
        ros::Time ts = scan->packets.end()->stamp;
        pcl::PointCloud<PointT> cloud, cloud_filtered;

        if (scan->packets.size() == 0)
        {
            cout << "No messages in velodyne package - Something wrong with sensor driver?" << endl;
            return;
        }
        bool unwarped = true;
        if (disable_compensation)
            ndt_generic::UnwarpCloudSimple(dataParser, scan, cloud);
        else
            unwarped = this->OdometryTFaUnwarpTransform(scan, cloud);

        if (cloud.size() == 0)
        {
            cout << "Cloud empty after unwarping - Problems combining odometry with lidar" << endl;
            return;
        }

        if (unwarped)
        {
            ndt_generic::filter_range_fun(cloud_filtered, cloud, min_range, max_range);
            cloud_filtered.header.frame_id = output_points_link;
            cloud_pub.publish(cloud_filtered);
            // cloud_filtered.clear();
            // 0cloud.clear();
            // cout<<"Delay unwarp: "<<ros::Time::now()-t0<<endl;
        }
    }

    bool Transform(const ros::Time& ts, Eigen::Affine3d& T)
    {
        tf::StampedTransform transform;
        tf_listener.waitForTransform(odom_link, odom_child_link, ts, ros::Duration(0.1));
        try
        {
            tf_listener.lookupTransform(odom_link, odom_child_link, ts, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }
        tf::poseTFToEigen(transform, T);
        return true;
    }

    template <class PointT>
    bool OdometryTFaUnwarpTransform(const velodyne_msgs::VelodyneScan::ConstPtr& msg,
                                    pcl::PointCloud<PointT>& cloud)
    {
        velodyne_msgs::VelodyneScan::ConstPtr scan = msg;

        cloud.clear();
        ros::Time t_first = scan->packets[0].stamp;
        ros::Time t_last = scan->packets[msg->packets.size() - 1].stamp;
        Eigen::Affine3d tf_first, tf_last;
        Eigen::Affine3d Todom_first, Todom_last;

        if (!Transform(t_first, Todom_first) || !Transform(t_last, Todom_last))
        { // tf exists for first and last
            std::cerr << "Tf error -  no unwarp" << std::endl;
            ndt_generic::UnwarpCloudSimple(dataParser, scan, cloud);
            cloud.header.frame_id = output_points_link;
            pcl_conversions::toPCL(t_last, cloud.header.stamp);
            return false;
        }
        else
        {
        }

        Eigen::Affine3d Tsens_last = Todom_last * Tsensor;
        static velodyne_pointcloud_oru::PointcloudXYZIR pnts(130, 0, "/velodyne", "/velodyne",
                                                             dataParser.scansPerPacket(), NULL);
        pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> cloud_aggregated;
        for (size_t next = 0; next < msg->packets.size(); ++next)
        {
            pnts.setup(scan);
            dataParser.unpack(scan->packets[next], pnts); // unpack the raw data
            pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> pnts_pcl, pnts_conv;
            ndt_generic::VelodyneToPcl(pnts, pnts_pcl);
            ros::Time t_pkg_i = scan->packets[next].stamp;
            Eigen::Affine3d Todom_i;
            Transform(t_pkg_i, Todom_i);
            Eigen::Affine3d Tsensor_tnow = Todom_i * Tsensor;
            Eigen::Affine3d Tmotion_sensor = Tsens_last.inverse() * Tsensor_tnow;

            tf::Transform Tmot_tf;
            tf::poseEigenToTF(Tmotion_sensor, Tmot_tf);
            pcl_ros::transformPointCloud(pnts_pcl, pnts_conv, Tmot_tf);
            cloud_aggregated += pnts_conv;
        }
        ndt_generic::convertPointCloud(cloud_aggregated, cloud);

        cloud.header.frame_id = output_points_link;
        pcl_conversions::toPCL(t_last, cloud.header.stamp);
        cloud.width = cloud.size();
        cloud.height = 1;
        return true;
    }

public:
    localisation_node(ros::NodeHandle param)
    {

        param.param<bool>("disable_compensation", disable_compensation, "false");
        param.param<std::string>("input_points_topic", input_points_topic, "/velodyne_packets");
        param.param<std::string>("sensor_offset_topic", sensor_offset_topic,
                                 "/calibrated_sensor_pose");
        param.param<std::string>("odom_link", odom_link, "/odom");
        param.param<std::string>("odom_child_link", odom_child_link, "/base_link");
        param.param<std::string>("output_points_link", output_points_link, "/velodyne");
        param.param<std::string>("output_points_topic", output_points_topic, "cloud_undistorted");

        Eigen::Vector3d sensor_offset_pos, sensor_offset_euler;
        param.param("sensor_pose_x", sensor_offset_pos(0), 0.);
        param.param("sensor_pose_y", sensor_offset_pos(1), 0.);
        param.param("sensor_pose_z", sensor_offset_pos(2), 0.);
        param.param("sensor_pose_r", sensor_offset_euler(0), 0.);
        param.param("sensor_pose_p", sensor_offset_euler(1), 0.);
        param.param("sensor_pose_t", sensor_offset_euler(2), 0.);
        Tsensor = ndt_generic::vectorsToAffine3d(sensor_offset_pos, sensor_offset_euler);

        param.param("min_range", min_range, 1.3);
        param.param("max_range", max_range, 130.0);

        dataParser.setup(param);
        dataParser.setParameters(min_range, max_range, 0, 2 * 3.1415);

        sensor_offset_sub =
            nh.subscribe(sensor_offset_topic, 1, &localisation_node::SensorOffsetCallback, this);
        std::cout << "Sensor translation: (" << sensor_offset_pos << "), euler angles: ("
                  << sensor_offset_euler << ")" << std::endl;

        bool use_pointtype_xyzir;
        param.param<bool>("use_pointtype_xyzir", use_pointtype_xyzir, "false");

        std::cout << "Input topic \"" << input_points_topic << "\"" << std::endl;
        std::cout << "Output topic \"" << output_points_topic << "\"" << std::endl;

        if (use_pointtype_xyzir)
        {
            cout << "Use xyzir" << endl;
            cloud_pub = nh.advertise<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>>(
                output_points_topic, 20);
            cloud_sub = nh.subscribe(
                input_points_topic, 10,
                &localisation_node::VeloCallback<velodyne_pointcloud_oru::PointXYZIR>, this);
        }
        else
        {
            cout << "Use pcl:PointXYZ" << endl;
            cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>(output_points_topic, 20);
            cloud_sub = nh.subscribe(input_points_topic, 10,
                                     &localisation_node::VeloCallback<pcl::PointXYZL>, this);
        }

        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mcl");
    ros::NodeHandle parameters("~");
    localisation_node pf(parameters);
}
