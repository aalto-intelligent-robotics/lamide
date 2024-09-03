#include "eigen_conversions/eigen_msg.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "laser_geometry/laser_geometry.h"
#include "ndt_map_lamide/pointcloud_utils.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

#include <Eigen/Eigen>
#include <graph_map_lamide/ndt_dl/point_curv.h>
#include <graph_map_lamide/ndt_dl/point_curv3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


using namespace perception_oru;

ros::Publisher pub, cloud_pub, cloud_flat_pub, cloud_less_flat_pub, cloud_corner_convex_pub,
    cloud_corner_concave_pub, cloud_less_corner_pub, cloud_edge_falling_pub, cloud_edge_rising_pub;

void Lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZL>> clouds;

    pcl::fromROSMsg(*msg, cloud);

    std::cout << "CALLBACK cloud size=" << cloud.size() << std::endl;
    Eigen::Affine3d offset = ndt_generic::xyzrpyToAffine3d(0, 0, 0, /*M_PI*/ 0, 0, 0);
    transformPointCloudInPlace(offset, cloud);

    // Do point curvature based segmentation

    double t0 = perception_oru::graph_map::getDoubleTime();
    perception_oru::graph_map::segmentPointCurvature3(offset, cloud, clouds);

    double t1 = perception_oru::graph_map::getDoubleTime();
    ROS_INFO_STREAM("[TestPointCurveNode]: segm time : " << t1 - t0);

    for (size_t i = 0; i < clouds.size(); i++)
    {
        clouds[i].header = cloud.header;
        std::cout << "clouds[" << i << "] size : " << clouds[i].size() << std::endl;
    }

    cloud_pub.publish(cloud);
    //  cloud_corner_convex_pub.publish(clouds[0]);
    //  cloud_less_corner_pub.publish(clouds[1]);
    //  cloud_flat_pub.publish(clouds[2]);
    cloud_less_flat_pub.publish(clouds[0]);
    cloud_edge_falling_pub.publish(clouds[1]);
    cloud_edge_rising_pub.publish(clouds[2]);

    if (!ros::ok())
        exit(0);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "test_point_curv");

    ros::NodeHandle n;

    cloud_pub =
        n.advertise<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>>("/points2_orig", 1);
    cloud_flat_pub = n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/cloud_flat", 1);
    cloud_less_flat_pub = n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/cloud_less_flat", 1);
    cloud_corner_convex_pub =
        n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/cloud_corner_convex", 1);
    cloud_less_corner_pub = n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/cloud_less_corner", 1);
    cloud_edge_falling_pub = n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/edge_falling", 1);
    cloud_edge_rising_pub = n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/edge_rising", 1);
    ros::Subscriber sub = n.subscribe("/points2", 1000, Lidar_callback);

    ros::spin();

    return 0;
}
