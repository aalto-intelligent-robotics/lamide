#ifndef NDTD2DREGTYPE_H
#define NDTD2DREGTYPE_H
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/reg_type.h"
#include "graph_map_lamide/map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include <ndt_map_lamide/ndt_map.h>
#include "ndt_map_lamide/lazy_grid.h"
#include "ndt_map_lamide/ndt_map_hmt.h"
#include "math.h"
#include <ndt_map_lamide/pointcloud_utils.h>
#include <ndt_registration_lamide/ndt_matcher_d2d_2d.h>
#include <ndt_registration_lamide/ndt_matcher_d2d.h>
#include <ndt_registration_lamide/ndt_matcher_d2d_sc.h>
#include "ndt_registration_lamide/ndt_matcher_d2d_sc_2d.h"
#include "graph_map_lamide/ndt/ndt_map_type.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "ros/publisher.h"
#include "ros/node_handle.h"
#include "pcl_ros/point_cloud.h"


#define ndt_d2d_reg_type_name "ndt_d2d_reg"

namespace perception_oru{
namespace graph_map{

class NDTD2DRegType:public registrationType{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~NDTD2DRegType();

  bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZL> &cloud, Eigen::MatrixXd &Tcov);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess

  bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow, std::vector<pcl::PointCloud<pcl::PointXYZL> > &clouds, Eigen::MatrixXd &Tcov);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess

  bool RegisterMap2Map(MapTypePtr map_prev,MapTypePtr map_next, Eigen::Affine3d &Tdiff,double &match_score);

  bool Register(pcl::PointCloud<pcl::PointXYZL> &target, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZL> &src, Eigen::MatrixXd &Tcov);

  std::string ToString();

  void SetLocalResolutionFactor(double resolution){resolutionLocalFactor_=resolution;}

  NDTMap* GetRegisteredMap() const{ return ndlocal;}

protected:

  NDTD2DRegType(RegParamPtr paramptr);

  NDTMatcherD2D_2D *matcher2D_;

  NDTMatcherD2D *matcher3D_;
  NDTMap *ndlocal=NULL;
  double resolution_=0.8,resolutionLocalFactor_=1.0;
  int  matcher2D_ITR_MAX_ = 35;
  bool matcher2D_step_control_=true;
  int  matcher2D_n_neighbours_=2;
  bool multires_=false;
  bool SoftConstraints_=false;
  bool plot_registration_=true;

private:



  friend class GraphFactory;
};



class NDTD2DRegParam:public registrationParameters{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~NDTD2DRegParam();

  void GetParametersFromRos();

  NDTD2DRegParam();

  double resolution=0.8, resolution_local_factor=1.0;
  //Matcher
  int  matcher2D_ITR_MAX = 35;
  bool matcher2D_step_control=true;
  int  matcher2D_n_neighbours=2;
  bool multires=false;
  bool SoftConstraints=false;

protected:

private:
  friend class GraphFactory;

};
}
}


#endif // NDTD2DREGTYPE_H

