
#include "eigen_conversions/eigen_msg.h"
#include "graph_localization_lamide/pose_queue.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "ndt_generic_lamide/eigen_utils.h"
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
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <time.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std;
using namespace perception_oru;
using namespace graph_localization;

ros::NodeHandle* nh;
void PlotPose(Eigen::Affine3d& T1)
{
    static ros::Publisher T1_pub = nh->advertise<nav_msgs::Odometry>("/T1", 500);
    ros::Time tplot = ros::Time::now();
    nav_msgs::Odometry transform_msg;
    tf::poseEigenToMsg(T1, transform_msg.pose.pose);
    transform_msg.header.frame_id = "/world";
    transform_msg.header.stamp = tplot;
    T1_pub.publish(transform_msg);
}

double t[] = {0, 0.2, 0.4, 0.6, 0.8, 1.0};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_localisation");
    nh = new ros::NodeHandle("~");
    PoseQueue p(5);
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    PlotPose(pose);
    sleep(3);
    for (int i = 0; i <= 5; i++)
    {

        Eigen::Vector3d trans(i, i, i);
        Eigen::Vector3d rot(0, 0, (double)i / 2);
        Eigen::Affine3d pose = ndt_generic::vectorsToAffine3d(trans, rot);
        cout << "push pose at t=" << t[i] << endl;
        p.push(pose, t[i]);
    }
    p.ToString();
    double nr_poses = 50;
    for (int i = -1; i <= nr_poses + 1; i++)
    {
        Eigen::Affine3d tmp;
        double time = i / nr_poses;
        if (p.GetPose(time, tmp))
        {
            PlotPose(tmp);
            std::cout << "[" << tmp.translation().transpose() << "]"
                      << " t=" << time << std::endl;
        }
        else
            cout << "error getting pose at t=" << time << endl;
        usleep(1000 * 50);
    }
    cout << "finished" << endl;

    ros::spinOnce();

    return 0;
}
