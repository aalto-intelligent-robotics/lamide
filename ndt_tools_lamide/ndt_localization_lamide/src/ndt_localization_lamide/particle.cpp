#include "ndt_localization_lamide/particle.hpp"
#include <iostream>
//perception_oru::particle::particle(unsigned int particule_cloud_size){
//  probability=1 / particule_cloud_size;
//  likelihood=1;
//  pose=Eigen::Affine3d::Identity();
//}


perception_oru::particle::particle(double roll, double pitch, double yaw, double x, double y, double z){
  this->Set(roll,pitch,yaw,x,y,z);
  probability = 1;
  likelihood = 1;
  weight = 1;
}
perception_oru::particle::particle(double roll, double pitch, double yaw, double x, double y, double z, double p){
  this->Set(roll,pitch,yaw,x,y,z);
  likelihood = 1;
  weight = 1;
  probability = p;
}
perception_oru::particle::particle(Eigen::Affine3d pose_){
  pose=pose_;
  probability = 1;
  likelihood = 1;
  weight = 1;
}


void perception_oru::particle::Set(double roll,double pitch,double yaw,double x, double y, double z){
  // r_=roll;
  // p_=pitch;
  // t_=yaw;
  // x_=x;
  // y_=y;
  // z_=z;

  Eigen::Matrix3d orientation;
  orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      *Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      *Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d position(x,y,z);
  //  std::cout<<x<<" "<<y<<" "<<z<<std::endl;
  //pose=position*orientation;
  pose=Eigen::Affine3d::Identity();
  pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  pose.translation()<<x,y,z;
}

void perception_oru::particle::GetXYZ(double &x, double &y, double &z){

  Eigen::Vector3d tr = pose.translation();
  x = tr[0];
  y = tr[1];
  z = tr[2];
  //x=x_;
  //y=y_;
  //z=z_;
}

void perception_oru::particle::GetRPY(double &r,double &p,double &y){
  //r=r_;
  //p=p_;
  //y=t_;
  Eigen::Vector3d rot = pose.rotation().eulerAngles(0,1,2);
  r=rot[0];
  p=rot[1];
  y=rot[2];
}

Eigen::Affine3d perception_oru::particle::GetAsAffine(){
  return pose;
}

void perception_oru::particle::SetLikelihood(double l_){
  likelihood=l_;
}

void perception_oru::particle::SetWeight(double w_){
  assert(w_ != 0);
  weight=w_;
}

void perception_oru::particle::SetProbability(double p_){
  //	assert(p_ >= 0);
  //	assert(p_ <= 1);
  probability=p_;
}

double perception_oru::particle::GetLikelihood() const {
  return likelihood;
}
double perception_oru::particle::GetWeight() const {
  return weight;
}

double perception_oru::particle::GetProbability() const {
  //	assert(probability >= 0);
  //	assert(probability <= 1);
  return probability;
}
