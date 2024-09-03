#include "eigen_conversions/eigen_msg.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "laser_geometry/laser_geometry.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_map_lamide/NDTMapMsg.h"
#include "ndt_map_lamide/ndt_cell.h"
#include "ndt_map_lamide/ndt_conversions.h"
#include "ndt_map_lamide/pointcloud_utils.h"
#include "ndt_map_lamide/polar_ndt_map.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


using namespace perception_oru;
ros::Publisher *pub, *cloud_pub;
graph_map::graphVisualization* vis;

void Lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZL> cloud, cloud2;
    pcl::fromROSMsg(*msg, cloud);
    std::cout << "CALLBACK cloud size=" << cloud.size() << std::endl;
    Eigen::Affine3d offset = ndt_generic::xyzrpyToAffine3d(0, 0, 0, M_PI, 0, 0);
    transformPointCloudInPlace(offset, cloud);

    perception_oru::PolarNDTMap scan;
    scan.AddPointCloud(cloud);
    std::vector<perception_oru::NDTCell*> cells;
    scan.ComputeAllCells(cells);

    for (int i = 0; i <= 360 && ros::ok(); i += 20)
    {
        double yaw = (double)i * M_PI / 180.0;
        scan.GetCloud(yaw, yaw, cloud2);
        ndt_map_lamide::NDTMapRGBMsg ndt_msg;
        toMessage(cells, ndt_msg, "world");
        cloud2.header.frame_id = "world";
        pcl_conversions::toPCL(ros::Time::now(), cloud2.header.stamp);
        pub->publish(ndt_msg);
        std::cout << "Publish could" << cloud2.size() << std::endl;
        cloud_pub->publish(cloud2);
        cloud2.clear();
        char c;
        std::cin >> c;
    }

    if (!ros::ok())
        exit(0);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "polar_ndt");

    ros::NodeHandle n;

    pub = new ros::Publisher();
    cloud_pub = new ros::Publisher();
    *pub = n.advertise<ndt_map_lamide::NDTMapRGBMsg>("scan", 1);
    *cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZL>>("/points2", 1);
    ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, Lidar_callback);

    ros::spin();

    return 0;
}
