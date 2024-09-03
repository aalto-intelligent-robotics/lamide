#pragma once
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <cmath>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/Imu.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "eigen_conversions/eigen_msg.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "stdio.h"
#include <iostream>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include <queue>
#define foreach BOOST_FOREACH

namespace ndt_offline{
class imu_prediction{
public:


  imu_prediction(const std::string &bag_path,Eigen::Affine3d sensor_offset=Eigen::Affine3d::Identity(), ros::NodeHandle *n=NULL,const Eigen::Affine3d &Tinit_pose=Eigen::Affine3d::Identity(),double init_time=0);

  void SetPose(Eigen::Affine3d &pose){}

  void prediction(Eigen::Affine3d &pose, double t_now);

  Eigen::Affine3d PredictOrientation(double tp1);

  Eigen::Affine3d Precict2(double tp1);

  Eigen::Affine3d PredictPoseAndVelocity(double tp1);

  void predict_pose(Eigen::Affine3d &pose, Eigen::Vector3d &vel, double t_prev, double t_now);

  void plotPose(const Eigen::Affine3d & T_pred, const Eigen::Vector3d &a1, const Eigen::Vector3d &a2);

  void update_registered_pose(const Eigen::Affine3d &pose, double t_now);

  void update_weights(Eigen::Affine3d pose, double t);

  void update_pose(Eigen::Affine3d &pose, Eigen::Vector3d &vel, sensor_msgs::Imu msg1, sensor_msgs::Imu msg2, bool plot);

  void update_pose2(Eigen::Affine3d &pose_w, Eigen::Vector3d &vel_w, sensor_msgs::Imu msg_integrate, double t_prev, double t_now);

  void rotate_vector(Eigen::Vector3d &v,Eigen::Quaterniond transform);

  Eigen::Vector3d avg_acc(sensor_msgs::Imu msg1, sensor_msgs::Imu msg2);

  sensor_msgs::Imu interpolate_imu_msg(double t, int index);

  Eigen::Quaterniond relative_orientation(geometry_msgs::Quaternion msg1, geometry_msgs::Quaternion msg2);

  Eigen::Quaterniond GetOrientationDiff(double t1, double t2);

  Eigen::Affine3d GetOrientationDiffAffine(double t1, double t2); //assuming zero motion of lidar

  Eigen::Quaterniond GetOrientation(double t2);

  Eigen::Affine3d GetOrientationAffine(double t);

  geometry_msgs::Quaternion slerp_quat(geometry_msgs::Quaternion msg1, geometry_msgs::Quaternion msg2, double k);//SLERP between 2 quat

  sensor_msgs::Imu custom_imu_msg(double foo);

  Eigen::Vector3d integrate(Eigen::Vector3d &v, double dt){return v*dt;}

  /*geometry_msgs::Pose transform_to_IMU(){}*/

  /*geometry_msgs::Pose transform_to_LIDAR(){}*/

  int find_imu_msg(double time);

  bool read_bag(const std::string &bag_path);

  void fill_buffer_custom(double nr);

  void display_buffer(int acc, int t_stamp, int orientation);


  double find_v(Eigen::Vector3d avg_v, double t_now);

  double v_comparison(Eigen::Vector3d v1, Eigen::Vector3d v2);

  void PredictVelocity(const Eigen::Quaterniond &qinit,double tinit ,Eigen::Vector3d &vel, double t);

private:
  double wx=1.0,wy=1.0,wz=1.0;
  double t_prev_=0;
  double old_t_now_=0;
  double t_offset=0;
  double alpha=0.8;
  Eigen::Vector3d previous_vel_ = {0,0,0};
  Eigen::Vector3d predicted_vel_= {0,0,0};
  Eigen::Vector3d average_vel_= {0,0,0};
  Eigen::Vector3d old_predicted_v_= {0,0,0};

  Eigen::Vector3d vel_t = {0,0,0};
  Eigen::Vector3d pred_vel_tp1= {0,0,0};
  Eigen::Affine3d pose_t;
  Eigen::Affine3d pose_tm1;
  double t,tm1;

  std::vector<sensor_msgs::Imu> imu_buffer;
  ros::NodeHandle *n_=NULL;
  Eigen::Affine3d Tsensor_;
//  ros::NodeHandle nh_;
};

}
//void imu_callback(const sensor_msgs::ImuConstPtr& msg)
//{
//    static  ros::Time Tprevmsg=msg->header.stamp;
//    static tf::TransformBroadcaster br;
//    Eigen::Vector3d offset;
//    Eigen::Quaterniond q;
//    Eigen::Vector3d accel;

//    tf::quaternionMsgToEigen(msg->orientation,q);
//    tf::vectorMsgToEigen( msg->linear_acceleration,accel);
//    if(counter++==0){
//        pose.linear()= q.toRotationMatrix();
//        pose.translation()=Eigen::Vector3d(0,0,0);
//        speed<<0,0,0;
//        //tf::vectorMsgToEigen( msg->linear_acceleration,offset);
//    }
//    else{
//        tf::vectorMsgToEigen( msg->linear_acceleration,accel);


//        double dt=(msg->header.stamp-Tprevmsg).toSec();
//        pose.translation()=pose.translation()+dt*speed;
//        speed=speed+dt*(accel-offset);
//        pose.linear()=q.toRotationMatrix();
//        if(speed.norm()>1){
//            std::cout<<"accel:"<<accel<<", norm:"<<accel.norm() <<std::endl;
//            //  exit(0);
//        }
//    }
//    std::cout<<"speed:"<<speed.transpose()<<std::endl;
//    std::cout<<"pos:"<<pose.translation().transpose()<<std::endl;
//    //std::cout<<"pose:\n"<<pose.rotation()<<std::endl;
//    tf::Transform transform;
//    tf::poseEigenToTF(pose,transform);
//    Tprevmsg=msg->header.stamp;
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "xsens"));
//    // if(counter%1000==0)
//    // speed<<0,0,0;
//}

