#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>
#include <ros/ros.h>
#include "ndt_offline_lamide/readpointcloud.h"
#include "eigen3/Eigen/Eigen"
#include <boost/program_options.hpp>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//#include "velodyne_pointcloud/transform.h"
//#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_pointcloud/point_types.h"
namespace po = boost::program_options;
using namespace std;
using namespace ndt_offline;
//using namespace velodyne_pointcloud;
//using namespace velodyne_rawdata;
int main(int argc, char **argv){
  ros::init(argc, argv, "graph_fuser3d_offline");
  string velodyne_config_file;
  std::string bag_file_path;
  std::string lidar_topic;
  std::string interp_link;
  std::string odom_type;
  ndt_offline::OdometryType odom;
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("bag-file" ,po::value<string>(&bag_file_path)->default_value(std::string("")) ,"bag file")
      ("velodyne-config-file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("../config/velo32.yaml")), "configuration file for the scanner")
      ("lidar-topic", po::value<std::string>(&lidar_topic)->default_value(std::string("/velodyne_packets")), "topic for velodyne topic")
      ("interpolation-link-id", po::value<std::string>(&interp_link)->default_value(std::string("odom_base_link")), "topic for velodyne topic")
      ("odom-type", po::value<std::string>(&odom_type)->default_value(std::string("")), "odom_type")
      ;


  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")){
    cout << desc << "\n";
    return 1;
  }
  Eigen::Affine3d Tsens=Eigen::Affine3d::Identity();
  pcl::PointCloud<pcl::PointXYZL> cloud;
  /*cout<<"Opening reader at path: "<<bag_file_path<<endl;
  if(odom_type=="WHEEL_ODOM")
    odom=WHEEL_ODOM;
  else if(odom_type=="IMU")
    odom=IMU;
  else
    odom=NO_ODOM;

  ndt_offline::readPointCloud *reader=new ndt_offline::readPointCloud(bag_file_path, Tsens,odom,lidar_topic,0.6,130,velodyne_config_file,0.0,"/tf","/world",interp_link);
  int counter=0;
  while(reader->readNextMeasurement(cloud) && ros::ok()){
    cout<<"Counter="<<counter++<<", Cloud size="<<cloud.size()<<endl;
    //usleep(1000*300);
  }
  cout<<"end of bag"<<endl;


*/

  rosbag::Bag bag;
  rosbag::View *view_;


  std::vector<string> topics;
  topics.push_back(lidar_topic);
  bag.open(bag_file_path, rosbag::bagmode::Read);
  view_ = new rosbag::View(bag, rosbag::TopicQuery(topics));
  std::cout<<"Messages found :"<<view_->size()<<std::endl;
  if(view_->size()==0)
    exit(0);

  /*boost::shared_ptr<velodyne_rawdata::RawData> data_( new velodyne_rawdata::RawData());
  data_->setupOffline(velodyne_config_file,0,1000);

  for(rosbag::View::iterator I = view_->begin(); I!=view_->end();I++){
    rosbag::MessageInstance const m = *I;
    if (velodyne_msgs::VelodyneScan::ConstPtr scan = m.instantiate<velodyne_msgs::VelodyneScan>()){
*/

      /*velodyne_rawdata::VPointCloud inPc_;

      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      inPc_.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      inPc_.header.frame_id = scanMsg->header.frame_id;
      inPc_.height = 1;
      inPc_.points.clear();
      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
        data_->unpack(scanMsg->packets[i], *inPc_);
    }
  }
*/












}
