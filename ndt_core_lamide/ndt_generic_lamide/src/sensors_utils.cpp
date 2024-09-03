#include "ndt_generic_lamide/sensors_utils.h"

namespace ndt_generic {
bool GetSensorPose(const std::string &dataset,  Eigen::Vector3d & transl,  Eigen::Vector3d &euler,tf::Transform &Tsensor_tf, Eigen::Affine3d &Tsensor_eig){

  tf::Quaternion quat;

  bool found_sensor_pose=false;
  if(dataset.compare("oru-field")==0){
    transl[0]=0;
    transl[1]=0;
    transl[2]=0;
    euler[0]=-0.00341974;
    euler[1]=0.00417214;
    euler[2]=-1.56988;
    found_sensor_pose=true;
    //    -0.0344436 --Euler-x -0.00341974 --Euler-y 0.00417214 --Euler-z -1.56988
  }
  else if(dataset.compare("oru-basement")==0){
    transl[0]=0.3;
    transl[1]=0;
    transl[2]=1.3;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.57;
    found_sensor_pose=true;
  }
  else if(dataset.compare("default")==0){
    transl[0]=0;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=0;
    found_sensor_pose=true;
  }
  else if(dataset.compare("arla-2012")==0){
    transl[0]=1.17920618535632;
    transl[1]= -0.285884882359476;
    transl[2]=2.0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.58804135060281;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("arla-2012-noisy")==0){
    transl[0]=1.17920618535632;
    transl[1]= -0.285884882359476;
    transl[2]=2.0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.58804135060281;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("michigan")==0){
    transl[0]=0.002;
    transl[1]= -0.004;
    transl[2]=-0.957;
    euler[0]=0.807*M_PI/180;
    euler[1]=0.166*M_PI/180;
    euler[2]=-90.703*M_PI/180;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("michigan-noisy")==0){
    transl[0]=0.002;
    transl[1]= -0.004;
    transl[2]=-0.957;
    euler[0]=0.807*M_PI/180;
    euler[1]=0.166*M_PI/180;
    euler[2]=-90.703*M_PI/180;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("coop-2013")==0){
    transl[0]=0.959 ;
    transl[1]= 0.343;
    transl[2]=0.022 ;
    euler[0]=0.0;//0.0038;
    euler[1]=0.0;
    euler[2]=0.121;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("kitti")==0){ //car center to lidar
    transl[0]=-0.27;
    transl[1]= 0;
    transl[2]=0.8;
    euler[0]=0;
    euler[1]=0;
    euler[2]=0;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("volvo_2017_12_01")==0){
    transl[0]= 0.270054;
    transl[1]= -0.000547494;
    transl[2]=3.79621;
    euler[0]=-0.00697922;
    euler[1]=-0.00933762;
    euler[2]=-3.11258 ;
    found_sensor_pose=true;
  }
  else if(dataset.compare("oru-lab")==0){
    transl[0]=0.807;
    transl[1]=-0.003;
    transl[2]=1.5;
    euler[0]=0.0;
    euler[1]=0.0;
    euler[2]=3.14;
    found_sensor_pose=true;
  }
  else if(dataset.compare("orkla-velodyne")==0){
    transl[0]=1.1187;
    transl[1]=0.1354;
    transl[2]=1.75;
    euler[0]=0.017;
    euler[1]=0.01;
    euler[2]=1.594682713;
    found_sensor_pose=true;
  }


  quat.setRPY(euler[0], euler[1], euler[2]);
  tf::Vector3 trans(transl[0], transl[1], transl[2]);
  Tsensor_tf  = tf::Transform(quat,trans);
  tf::poseTFToEigen(Tsensor_tf,Tsensor_eig);

  return found_sensor_pose;
}
}

