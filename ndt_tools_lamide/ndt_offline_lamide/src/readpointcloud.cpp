#include "ndt_offline_lamide/readpointcloud.h"
namespace ndt_offline{
//readPointCloud::readPointCloud(const std::string &bag_path){bag_path_=bag_path;}
readPointCloud::readPointCloud(const std::string &bag_path, const Eigen::Affine3d &T_sensor,const OdometryType odometry_sensor, const std::string &lidar_topic, double min_range, double max_range,const std::string &velodyne_config, double t_offset, const std::string &tf_topic, const std::string &fixed_frame_id, const std::string &interpolation_link_id, bool unpack_raw):nh("~"){
  odometry_sensor_=odometry_sensor; //can be either wheel odometry or imu

    min_range_ = unpack_raw ? -1 : min_range;
    max_range_ = unpack_raw ? 1000 : max_range;

  t_offset_=t_offset;
  Tsensor_=T_sensor;
  lidar_topic_=lidar_topic;
  interpolation_link_id_=interpolation_link_id;

  std::vector<string> topics{lidar_topic_,tf_topic};
  bag_.open(bag_path, rosbag::bagmode::Read);

  for(int i=0; i<topics.size(); ++i)
    std::cout<<"Searched Topic ["<<i<<"]="<<topics[i].c_str()<<std::endl;

  view_ = new rosbag::View(bag_, rosbag::TopicQuery(topics));
  std::cout<<"Messages found :"<<view_->size()<<std::endl;
  if(view_->size()==0)
    exit(0);

  I = view_->begin();

  if(odometry_sensor_ == IMU)
    imu_pred_=new imu_prediction(bag_path);

  else if(odometry_sensor_ == WHEEL_ODOM|| odometry_sensor_ == NO_ODOM)
    odom_lookup = new PoseInterpolationNavMsgsOdo(view_, tf_topic, fixed_frame_id, ros::Duration(3600), NULL);


  if(velodyne_config!=""){
    velodyne_parser_=new velodyne_rawdata_oru::RawData();
    velodyne_parser_->setupOffline(velodyne_config, max_range_, min_range_);
    velodyne_parser_->setParameters(min_range, max_range ,-0.000001 , 2*M_PI+0.000001);
    //velodyne_parser_->setParameters(min_range, max_range, 0,2*M_PI);
  }
  else
    std::cerr<<"Velodyne configuration file not loaded"<<std::endl;
}

ros::Time readPointCloud::GetTimeOfLastCloud(){
  return t_cloud;
}
bool readPointCloud::SkipToCloudIndex(const unsigned int index){
  bool found = false;
  unsigned int current = 0;
  rosbag::View::iterator I_tmp = view_->begin();
  while(I_tmp != view_->end() ){
    rosbag::MessageInstance const m = *I_tmp;
    if(m.getTopic()==lidar_topic_){
      if(current == index){
        found = true;
        I = I_tmp;
        break;
      }
      current++;
    }
    I_tmp++;
  }
  return found;
}

bool readPointCloud::GetOdomPose(const ros::Time &t, const std::string &frame_id,Eigen::Affine3d &pose){
  if(odom_lookup!=NULL)
    return odom_lookup->getTransformationForTime(t, frame_id, pose);
  else
    std::cerr<<"Cannot look up pose unless WHEEL_ODOM is specified"<<std::endl;
  return false;
}
bool readPointCloud::GetOdomPose(const ros::Time &t, const std::string &frame_id, const std::string &fixed_frame_id, Eigen::Affine3d &pose){
  if(odom_lookup!=NULL)
    return odom_lookup->getTransformationForTime(t, frame_id,fixed_frame_id, pose);
  else
    std::cerr<<"Cannot look up pose unless WHEEL_ODOM is specified"<<std::endl;
  return false;
}


/*void readPointCloud::plotPoses(const Eigen::Affine3d & T1, const Eigen::Affine3d & T2){

  static ros::Publisher T1_pub=nh.advertise<nav_msgs::Odometry>("/T1", 50);
  static ros::Publisher T2_pub=nh.advertise<nav_msgs::Odometry>("/T2", 50);
  ros::Time tplot=ros::Time::now();
  nav_msgs::Odometry transform_msg;
  tf::poseEigenToMsg(T1,transform_msg.pose.pose);
  transform_msg.header.frame_id="/world";
  transform_msg.header.stamp=tplot;
  T1_pub.publish(transform_msg);
  tf::poseEigenToMsg(T2,transform_msg.pose.pose);
  T2_pub.publish(transform_msg);
}*/

}
