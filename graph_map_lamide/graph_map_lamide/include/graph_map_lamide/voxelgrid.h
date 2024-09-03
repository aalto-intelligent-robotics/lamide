
//#ifndef VOXEL_GRID_H
//#define VOXEL_GRID_H
//#include <string>
//#include <vector>
//#include <numeric>
//#include <cmath>
//#include <algorithm>
//#include <stdexcept>
//#include <sstream>
//#include <sys/time.h>
//#include <time.h>
//#include <iostream>
//#include <Eigen/Dense>
//#include "map"
//#include "ndt_generic_lamide/io.h"
//#include <cmath>
//#include "iostream"
//#include "ndt_generic_lamide/io.h"
//#include "ndt_generic_lamide/serialization.h"
//#include "boost/serialization/serialization.hpp"
//#include "boost/serialization/vector.hpp"
//#include <boost/serialization/map.hpp>
// namespace perception_oru{
// namespace graph_map{
// template <class T>
// class VoxelGrid{
// public:

//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// VoxelGrid(const Eigen::Vector3d &resolution, const Eigen::Vector3d &center){
// resolution_=resolution; center_=center; initialized_=true;} VoxelGrid(){ initialized_=false;}
// void Initialize(const Eigen::Vector3d &resolution, const Eigen::Vector3d &center){
//   resolution_=resolution;
//   center_=center;
//   initialized_=true;
// }

// void SetVal(T var, const Eigen::Vector3d &pos){
//   if(!initialized_){
//     std::cerr<<"Voxel grid not initialized"<<std::endl;
//     exit(0);
//   }
//   std::string key=ndt_generic::ToString(GetIndex(pos));
//   data_[key]=var;
//   //std::cout<<"Set val: \""<<var<<"\" at with key= \""<<key<<"\""<<std::endl;
// }
// bool GetVal(T &t,const Eigen::Vector3d &pos){
//   if(!initialized_){
//     std::cerr<<"Voxel grid not initialized"<<std::endl;
//     exit(0);
//   }

//  std::string key=ndt_generic::ToString(GetIndex(pos));
//  if ( data_.find(key) != data_.end() ){
//    t=data_.at(key);
//    return true;
//  }
//  else
//    return false;
//}
// Eigen::Vector3i GetIndex(const Eigen::Vector3d &pos){

//  Eigen::Vector3i index;

//  for(int i=0;i<3;i++){

//    double diff=(pos(i)-center_(i));
//    if(fabs(diff)<=resolution_(i)/2.0){
//      index(i)=0;
//    }
//    else{
//      if(diff>=0){
//        index(i)=ceil((diff-(resolution_(i)/2.0))/resolution_(i));
//      }
//      else{
//        double tmp=diff+resolution_(i)/2.0;
//        index(i)=floor((diff+(resolution_(i)/2.0))/resolution_(i));
//      }
//    }
//  }
//  return index;
//}
// protected:
//  bool initialized_=false;
//  std::map<std::string,T> data_;
//  Eigen::Vector3d center_;
//  Eigen::Vector3d resolution_;
// private:
//  friend class boost::serialization::access;
//  template<class Archive>
//  void serialize(Archive & ar, const unsigned int version)
//  {
//    ar & initialized_;
//    ar & data_;
//    ar & center_;

//    ar & resolution_;
//  }

//};
//}
//}

//#endif // VOXEL_GRID_H
