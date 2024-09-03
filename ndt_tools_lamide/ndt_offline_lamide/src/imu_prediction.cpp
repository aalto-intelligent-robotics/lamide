#include "ndt_offline_lamide/imu_prediction.h"

namespace ndt_offline{
imu_prediction::imu_prediction(const std::string &bag_path, Eigen::Affine3d sensor_offset, ros::NodeHandle *n,const Eigen::Affine3d &Tinit_pose,double init_time){
  read_bag(bag_path);
  t_prev_=imu_buffer[0].header.stamp.toSec();
  old_t_now_=imu_buffer[0].header.stamp.toSec();
  n_= n;
  Tsensor_=sensor_offset;
  old_t_now_=t_prev_=init_time;

  pose_t=Tinit_pose;
  pose_tm1=Tinit_pose;
  tm1=init_time-0.1;
  t=init_time;

  //    Eigen::Quaterniond q_test = {0.5,0.5,0.5,0.5};
  //    Eigen::Vector3d v_test = {1,0,0};
  //    std::cout<<"rotating v: "<<std::endl<<v_test<<" so that x becomes y, ie 90deg around z."<<std::endl;
  //    rotate_vector(v_test,q_test);
  //    std::cout<<"rotated v: "<<std::endl<<v_test<<std::endl;
  //    std::cout<<"rotating v: "<<std::endl<<v_test<<" so that y becomes x, ie -90deg around z."<<std::endl;
  //    q_test = q_test.inverse();
  //    rotate_vector(v_test,q_test);
  //    std::cout<<"rotated v: "<<std::endl<<v_test<<std::endl;
  //    std::ofstream reg_pos;
  //    reg_pos.open("imu_data.txt",std::ios::app);
  //    if (reg_pos.is_open())
  //    {
  //        for (int i=0;i<imu_buffer.size();i++){
  //            reg_pos<<imu_buffer[i].header.stamp<<"\n";
  //            reg_pos<<imu_buffer[i].linear_acceleration<<"\n";
  //            reg_pos<<imu_buffer[i].orientation<<"\n"<<"\n";
  //        }
  //        reg_pos.close();
  //    }
  //    else
  //    {
  //        std::cout << "Failed to open the file!"<<std::endl;
  //    }
}

void imu_prediction::prediction(Eigen::Affine3d &pose, double t_now)
{
  //    std::cout<<"linear part of tmotion input: "<<pose.linear()<<std::endl;
  //    std::cout<<"translation: "<<pose.translation()<<std::endl;
  //    std::cout<<"rotation of tmotion input: "<<pose.rotation()<<std::endl;
  //    std::cout<<"the time received as input for prediction: "<<t_now<<std::endl;
  //    std::cout<<"the time of the first imu msg in buffer: "<<imu_buffer[0].header.stamp.toSec()<<std::endl;
  //    std::cout<<std::endl;

  /*predicted_vel_=previous_vel_; //DANIEL KOMMENTERADE DETTA
  predict_pose(pose,predicted_vel_,t_prev_,t_now);
  p_t=pose*p_tm1;
  std::cout<<"the predicted pose was made with a dt of "<<t_now-t_prev_<<std::endl;*/
}
Eigen::Affine3d imu_prediction::Precict2(double tp1){

  Eigen::Affine3d pred_pose_tp1=pose_t;

  int index_after_first=find_imu_msg(t);
  int index_after_last =find_imu_msg(tp1);

  for (int i=index_after_first;i<index_after_last;i++)//all imu msgs between the two interpolated msgs
    update_pose2(pred_pose_tp1, vel_t,imu_buffer[i], imu_buffer[i].header.stamp.toSec(), imu_buffer[i+1].header.stamp.toSec());


  return pose_t.inverse()*pred_pose_tp1; //return the predicted movement
}
void imu_prediction::PredictVelocity(const Eigen::Quaterniond &qinit,double tinit ,Eigen::Vector3d &vel, double t){

  Eigen::Quaterniond q=qinit;
  Eigen::Affine3d pose=Eigen::Affine3d::Identity();
  pose.linear()=q.toRotationMatrix();
  int index_after_first=find_imu_msg(tinit);
  int index_after_last =find_imu_msg(t);

  for (int i=index_after_first;i<index_after_last;i++)//all imu msgs between the two interpolated msgs
   update_pose2(pose, vel,imu_buffer[i],imu_buffer[i].header.stamp.toSec(),imu_buffer[i+1].header.stamp.toSec());

}
Eigen::Affine3d imu_prediction::PredictOrientation(double tp1){

  Eigen::Affine3d p_diff=Eigen::Affine3d::Identity();
  p_diff.linear()=Tsensor_.linear().inverse()*GetOrientationDiff(t,tp1)*Tsensor_.linear();
}
Eigen::Affine3d imu_prediction::PredictPoseAndVelocity(double tp1){

  double t_tm1=(t-tm1)/2; //approximate time of average velocity
  Eigen::Vector3d V_avg_t_tm1=((pose_t.translation()-pose_tm1.translation())/(t-tm1));

  //Eigen::Quaterniond orient_t_tm1=Eigen::Quaterniond(pose_tm1.linear()).slerp(0.5,Eigen::Quaterniond(pose_t.linear()));//.slerp(Eigen::Scaling(0.5),p_t.linear());
  //PredictVelocity(orient_t_tm1,t_tm1,V_avg_t_tm1,t); //

  pred_vel_tp1=alpha*vel_t+(1-alpha)*V_avg_t_tm1;
  Eigen::Affine3d pred_pose_tp1=pose_t;
  int index_after_first=find_imu_msg(t);
  int index_after_last =find_imu_msg(tp1);
  for (int i=index_after_first;i<index_after_last;i++)//all imu msgs between the two interpolated msgs
    update_pose2(pred_pose_tp1, pred_vel_tp1, imu_buffer[i], imu_buffer[i].header.stamp.toSec(), imu_buffer[i+1].header.stamp.toSec());

  return pose_t.inverse()*pred_pose_tp1;
  //return
}

void imu_prediction::update_registered_pose(const Eigen::Affine3d &pose, double t_pose){
  //when registration is succesful
  //    std::cout<<"Registration was apparently succesful. Entering update_registered_pose."<<std::endl;
  //    std::cout<<"About to update average_vel_. Dividing by t-now-old_t_now_ which equals: "<<t_now-old_t_now_<<std::endl;
  //update_weights(pose, t_pose);
  if (t_pose-old_t_now_ != 0){
    //        std::cout<<"Average vel is before update: "<<average_vel_<<std::endl;
    //        std::cout<<"Pose_old_.translation: "<<pose_old_.translation()<<std::endl;
    //        std::cout<<"pose.translation: "<<pose.translation()<<std::endl;
    //average_vel_ = ((pose_tm1.inverse()*pose).translation())/(t_pose-old_t_now_);
    //Eigen::Quaterniond q(pose.linear());
    //rotate_vector(average_vel_,q.inverse());
    //        average_vel_ = {0,0,0};
    //        std::cout<<"Average vel is after update: "<<average_vel_<<std::endl;
    pose_tm1=pose_t;
    pose_t=pose;
    vel_t=pred_vel_tp1;
    tm1=t;
    t=t_pose;
  }else{
    std::cout<<"Can't divide by zero. average_vel_ stays the same since we did not progress in time."<<std::endl;
    std::cout<<"Average vel is at the moment: "<<average_vel_<<std::endl;
  }
  //    std::cout<<"Starting to look for v."<<std::endl;
  t_prev_=find_v(average_vel_, t_pose);
  //    previous_vel_=(previous_vel_*0.2)+(average_vel_*0.8);
  previous_vel_=average_vel_;
  old_t_now_=t_pose;
  old_predicted_v_=predicted_vel_;
}
void imu_prediction::update_weights(Eigen::Affine3d pose, double t){
  Eigen::Vector3d translational_error=((pose_t.inverse()*pose).translation());//kan vara något fel här p_1 förut, nu pose_t
  Eigen::Quaterniond q=GetOrientation(t);
  rotate_vector(translational_error,q.inverse());
  wx=wx-translational_error[0]/10;
  wy=wy-translational_error[1]/10;
  wz=wz-translational_error[2]/10;
  std::cout<<"translational_error: "<<translational_error<<std::endl;
  std::cout<<"weights: "<<wx<<" "<<wy<<" "<<wz<<std::endl;

}
void imu_prediction::predict_pose(Eigen::Affine3d &pose, Eigen::Vector3d &vel, double t_prev, double t_now)
{

  int imu_index_1=find_imu_msg(t_prev-t_offset);
  int imu_index_2=find_imu_msg(t_now-t_offset);
  sensor_msgs::Imu ip_imu_msg_1 = interpolate_imu_msg(t_prev-t_offset, imu_index_1);
  sensor_msgs::Imu ip_imu_msg_2 = interpolate_imu_msg(t_now-t_offset, imu_index_2);
  Eigen::Quaterniond rel_ori=relative_orientation(ip_imu_msg_1.orientation,ip_imu_msg_2.orientation);

  update_pose(pose, vel,ip_imu_msg_1,imu_buffer[imu_index_1+1],true); //integrate from message  t_prev to  t(imu_index_1+1)plotPose(pose);
  for (int i=imu_index_1+1;i<imu_index_2;i++){//all imu msgs between the two interpolated msgs
    update_pose(pose, vel,imu_buffer[i], imu_buffer[i+1],true);
  }
  update_pose(pose, vel,imu_buffer[imu_index_2],ip_imu_msg_2,true);//integrate from message t(imu_index_2-1 to) t_now
  pose.linear()=rel_ori.toRotationMatrix();
}



double imu_prediction::find_v(Eigen::Vector3d avg_v, double t_now){
  //find the imu message between timestamps that are most similar to v
  std::vector<double> similarity_score;
  Eigen::Affine3d pose=Eigen::Affine3d::Identity();
  int lowest_score_index = 0;
  int imu_index_1=find_imu_msg(old_t_now_-t_offset);
  //    std::cout<<"Found imumsg1 which is: "<<imu_index_1<<". Will now look for imumsg2."<<std::endl;
  int imu_index_2=find_imu_msg(t_now-t_offset);
  //    std::cout<<"Found imumsg2: "<<imu_index_2<<". Identifying similarity score of all msgs between imumsg1 and imumsg2."<<std::endl;
  if (imu_index_1 == imu_index_2){
    //        std::cout<<"search was to be made between the two same index numbers. skipping search and returning value directly."<<std::endl;
    return imu_index_1;
  }else{
    for (int i=imu_index_1;i<imu_index_2;i++){//all imu msgs between the two timestamps
      //            std::cout<<"the old predicted v before "<<old_predicted_v_<<std::endl;
      update_pose(pose, old_predicted_v_,imu_buffer[i], imu_buffer[i+1], false);
      //            std::cout<<"the old predicted v and after "<<old_predicted_v_<<std::endl;
      //            std::cout<<"and the avg_v "<<avg_v<<std::endl;
      similarity_score.push_back(v_comparison(avg_v,old_predicted_v_));
    }
    //        std::cout<<"created similarity_score vector. moving on to see which one was lowest."<<std::endl;
    double lowest_score = similarity_score[similarity_score.size()-1];
    lowest_score_index = similarity_score.size();
    //        std::cout<<"lowest score: "<<lowest_score<<std::endl;
    for (int i=0;i<similarity_score.size();i++){
      //            std::cout<<"comparing with this score: "<<similarity_score[i]<<std::endl;
      if (similarity_score[i]<lowest_score){
        lowest_score=similarity_score[i];
        //                std::cout<<"lowest score: "<<lowest_score<<std::endl;
        lowest_score_index = i;
      }
    }
  }
  //the imu message that was most similar to the avg_v from registration
  //      return imu_index_1+lowest_score_index;
  //    std::cout<<"found the most similar imumsg to the avg_vel according to registration. "<<imu_index_1+lowest_score_index<<" was the imu_buffer index. avg_vel was "<<avg_v<<std::endl;
  return imu_buffer[imu_index_1+lowest_score_index].header.stamp.toSec();
  //            return imu_buffer[imu_index_2].header.stamp.toSec();

}

double imu_prediction::v_comparison(Eigen::Vector3d v1, Eigen::Vector3d v2){
  double score_x = v1[0]-v2[0];
  double score_y = v1[1]-v2[1];
  double score_z = v1[2]-v2[2];
  //    std::cout<<"the xyz scores "<<score_x<<" "<<score_y<<" "<<score_z<<std::endl;
  //    std::cout<<"the abs xyz scores "<<fabs(score_x)<<" "<<fabs(score_y)<<" "<<fabs(score_z)<<std::endl;
  double score_tot = fabs(score_x)+fabs(score_y)+fabs(score_z);
  return score_tot;
}

void imu_prediction::update_pose(Eigen::Affine3d &pose, Eigen::Vector3d &vel_,sensor_msgs::Imu msg1, sensor_msgs::Imu msg2, bool plot){
  double dt=msg2.header.stamp.toSec()-msg1.header.stamp.toSec();
  //    Eigen::Vector3d acc(msg1.linear_acceleration.x,msg1.linear_acceleration.y,msg1.linear_acceleration.z);
  Eigen::Vector3d acc(msg1.linear_acceleration.x*wx,msg1.linear_acceleration.y*wy,msg1.linear_acceleration.z*wz);
  Eigen::Quaterniond q(Tsensor_.linear());
  rotate_vector(acc,q);
  //vel_=vel_+(acc*dt);
  //    pose.translation()=pose.translation()+dt*vel_;
  pose.translate(vel_*dt);
  //    std::cout<<"updating pose.. displaying delta t "<<dt<<" and translation "<<pose.translation()<<" and linear "<<pose.linear()<<std::endl;
  //if (plot)plotPose(pose);

}
void imu_prediction::update_pose2(Eigen::Affine3d &pose_w, Eigen::Vector3d &vel_w, sensor_msgs::Imu msg_integrate, double t_prev, double t_msg_integrate){
  double dt=t_msg_integrate-t_prev;
  pose_w.linear()=pose_w.linear()*Tsensor_.linear().inverse()*GetOrientationDiff(t_prev,t_msg_integrate)*Tsensor_.linear();
  Eigen::Vector3d acc(msg_integrate.linear_acceleration.x,msg_integrate.linear_acceleration.y,msg_integrate.linear_acceleration.z);

  Eigen::Matrix3d R_world_imu= pose_w.linear()*Tsensor_.linear().inverse();
  Eigen::Vector3d acc_aligned=R_world_imu*acc;
  vel_w=vel_w+(acc_aligned*dt);
  pose_w.translation()=pose_w.translation()+vel_w*dt;

  //plotPose(pose_w,acc,acc_aligned);

}

void imu_prediction::rotate_vector(Eigen::Vector3d &v,Eigen::Quaterniond transform){
  transform.normalize();
  Eigen::Quaterniond p;
  p.w() = 0;
  p.vec() = v;
  Eigen::Quaterniond rotatedP = transform * p *  transform.inverse();
  v = rotatedP.vec();
}

void imu_prediction::plotPose(const Eigen::Affine3d & T_pred, const Eigen::Vector3d & a1,const Eigen::Vector3d & a2){
  static ros::Publisher Tpred_pub=n_->advertise<nav_msgs::Odometry>("/Tpred", 500);
  static ros::Publisher t_pub=n_->advertise<nav_msgs::Odometry>("/t", 500);
  static ros::Publisher tm1_pub=n_->advertise<nav_msgs::Odometry>("/tm1", 500);
  static ros::Publisher a1_pub=n_->advertise<sensor_msgs::Imu>("/a1", 500);
  static ros::Publisher a2_pub=n_->advertise<sensor_msgs::Imu>("/a2", 500);
  ros::Time tplot=ros::Time::now();
  nav_msgs::Odometry transform_msg;
  sensor_msgs::Imu imu;
  //T_pred
  tf::poseEigenToMsg(T_pred,transform_msg.pose.pose);
  transform_msg.header.frame_id="/world";
  transform_msg.header.stamp=tplot;
  Tpred_pub.publish(transform_msg);

  tf::poseEigenToMsg(pose_t,transform_msg.pose.pose);
  transform_msg.header.frame_id="/world";
  transform_msg.header.stamp=tplot;
  t_pub.publish(transform_msg);

  tf::poseEigenToMsg(pose_tm1,transform_msg.pose.pose);
  transform_msg.header.frame_id="/world";
  transform_msg.header.stamp=tplot;
  tm1_pub.publish(transform_msg);

  imu.header.stamp=tplot;
  imu.header.frame_id="world";
  imu.linear_acceleration.x=a1(0);
  imu.linear_acceleration.y=a1(1);
  imu.linear_acceleration.z=a1(2);
  a1_pub.publish(imu);
  imu.linear_acceleration.x=a2(0);
  imu.linear_acceleration.y=a2(1);
  imu.linear_acceleration.z=a2(2);
  a2_pub.publish(imu);
  int ms=5;
  usleep(ms*5000);
  //  int c=getchar();
}

Eigen::Vector3d imu_prediction::avg_acc(sensor_msgs::Imu msg1, sensor_msgs::Imu msg2){
  return Eigen::Vector3d( (msg1.linear_acceleration.x+msg2.linear_acceleration.x)/2,(msg1.linear_acceleration.y+msg2.linear_acceleration.y)/2,(msg1.linear_acceleration.z+msg2.linear_acceleration.z)/2);
}

sensor_msgs::Imu imu_prediction::interpolate_imu_msg(double t, int index){

  if(index<0 ||index+1>imu_buffer.size()){
    std::cerr<<"error accessing imu message out of range in time"<<std::endl;
    exit(0);
  }

  double dt = t - imu_buffer[index].header.stamp.toSec();
  double k = dt / (imu_buffer[index+1].header.stamp.toSec()-imu_buffer[index].header.stamp.toSec());
  sensor_msgs::Imu ip_imu_msg = imu_buffer[index];
  ip_imu_msg.header.stamp=ros::Time(t);
  ip_imu_msg.linear_acceleration.x=imu_buffer[index].linear_acceleration.x+(imu_buffer[index+1].linear_acceleration.x-imu_buffer[index].linear_acceleration.x)*k;
  ip_imu_msg.linear_acceleration.y=imu_buffer[index].linear_acceleration.y+(imu_buffer[index+1].linear_acceleration.y-imu_buffer[index].linear_acceleration.y)*k;
  ip_imu_msg.linear_acceleration.z=imu_buffer[index].linear_acceleration.z+(imu_buffer[index+1].linear_acceleration.z-imu_buffer[index].linear_acceleration.z)*k;
  ip_imu_msg.orientation=slerp_quat(imu_buffer[index].orientation,imu_buffer[index+1].orientation,k);
  return ip_imu_msg;

}
Eigen::Quaterniond imu_prediction::relative_orientation(geometry_msgs::Quaternion msg1, geometry_msgs::Quaternion msg2){

  Eigen::Quaterniond q1;
  Eigen::Quaterniond q2;
  tf::quaternionMsgToEigen(msg1,q1);
  tf::quaternionMsgToEigen(msg2,q2);
  Eigen::Quaterniond q3;
  q3 = q1.inverse()*q2;;//sure this is the correct order? which is first in time?
  return q3;

}

geometry_msgs::Quaternion imu_prediction::slerp_quat(geometry_msgs::Quaternion msg1, geometry_msgs::Quaternion msg2, double k){//SLERP between 2 quat
  Eigen::Quaterniond q1;
  Eigen::Quaterniond q2;
  Eigen::Quaterniond q3;
  tf::quaternionMsgToEigen(msg1,q1);
  tf::quaternionMsgToEigen(msg2,q2);
  q3 = q1.slerp(k,q2);
  geometry_msgs::Quaternion msg3;
  tf::quaternionEigenToMsg(q3,msg3);
  return msg3;
}

int imu_prediction::find_imu_msg(double time){
  int i = 0;
  if ( time < imu_buffer[0].header.stamp.toSec()){
    std::cout<<"time is before IMU started running"<<std::endl;
    return 0;
  }
  if ( time > imu_buffer[imu_buffer.size()-1].header.stamp.toSec()){
    std::cout<<"time is after IMU stopped running"<<std::endl;
    return imu_buffer.size()-1;
  }
  while ( time > imu_buffer[i+1].header.stamp.toSec() ) i++;
  return i;
}


Eigen::Quaterniond imu_prediction::GetOrientationDiff(double t1,double t2){
  sensor_msgs::Imu ip_imu_msg_1 = interpolate_imu_msg(t1, find_imu_msg(t1));
  sensor_msgs::Imu ip_imu_msg_2 = interpolate_imu_msg(t2, find_imu_msg(t2));
  return relative_orientation(ip_imu_msg_1.orientation,ip_imu_msg_2.orientation);
}
Eigen::Affine3d imu_prediction::GetOrientationDiffAffine(double t1,double t2){
  Eigen::Quaterniond q_t=GetOrientationDiff(t1,t2);
  Eigen::Affine3d T_rot=Eigen::Affine3d::Identity();
  T_rot.linear()=q_t.toRotationMatrix();
  return T_rot;
}
Eigen::Quaterniond imu_prediction::GetOrientation(double t){
  sensor_msgs::Imu msg = interpolate_imu_msg(t, find_imu_msg(t));
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(msg.orientation,q);
  return q;
}


Eigen::Affine3d imu_prediction::GetOrientationAffine(double t){
  Eigen::Quaterniond q_t=GetOrientation(t);
  Eigen::Affine3d T_rot=Eigen::Affine3d::Identity();
  T_rot.linear()=q_t.toRotationMatrix();
  return T_rot;
}

bool imu_prediction::read_bag(const std::string &bag_path){
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/imu/data"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
    if (s != NULL)
      imu_buffer.push_back(*s);
  }
  bag.close();
  std::cout<<"found :"<<imu_buffer.size()<<" imu messages."<<std::endl;
  return imu_buffer.size()>0; //return true if messages were read
}
void imu_prediction::display_buffer(int acc, int t_stamp, int orientation){
  for (int i=0; i<imu_buffer.size();i++){
    if (acc){
      std::cout<<"acc x:"<<imu_buffer[i].linear_acceleration.x<<" y:"<<imu_buffer[i].linear_acceleration.y<<" z:"<<imu_buffer[i].linear_acceleration.z<<std::endl;
    }
    if (t_stamp){
      std::cout<<"Time stamp: "<<imu_buffer[i].header.stamp<<std::endl;
    }
    if (orientation){
      std::cout<<"Quaternion:"<<imu_buffer[i].orientation.x<<" "<<imu_buffer[i].orientation.y<<" "<<imu_buffer[i].orientation.z<<" "<<imu_buffer[i].orientation.w<<std::endl;
    }
    std::cout<<std::endl;
  }
}
void imu_prediction::fill_buffer_custom(double nr){
  for (double i=0;i<nr;i++){
    imu_buffer.push_back(custom_imu_msg(i));
  }
}

sensor_msgs::Imu imu_prediction::custom_imu_msg(double foo){
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time(foo+1);
  msg.linear_acceleration.x=1;
  msg.linear_acceleration.y=0;
  msg.linear_acceleration.z=0;
  msg.orientation.x=0;
  msg.orientation.y=0;
  msg.orientation.z=0;
  msg.orientation.w=1;
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, foo*0.2);
  tf::quaternionTFToMsg(q,msg.orientation);
  return msg;
}

}
