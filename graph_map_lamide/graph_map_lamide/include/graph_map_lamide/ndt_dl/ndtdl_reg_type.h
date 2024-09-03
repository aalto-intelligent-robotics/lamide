#ifndef NDTDL_REG_TYPE_H
#define NDTDL_REG_TYPE_H

#include "Eigen/Dense"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/reg_type.h"
#include "math.h"
#include "ndt_generic_lamide/point_types.h"
#include "ndt_registration_lamide/ndt_matcher_d2d_n.h"
#include "ndt_registration_lamide/ndt_matcher_d2d_n_sc.h"
#include "pcl/io/pcd_io.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

#include <pcl/point_cloud.h>


#define ndt_dl_reg_type_name "ndt_dl_reg"
namespace perception_oru
{
namespace graph_map
{

class NDTDLRegType : public registrationType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTDLRegType();

    bool Register(MapTypePtr maptype,
                  Eigen::Affine3d& Tnow,
                  pcl::PointCloud<pcl::PointXYZL>& cloud,
                  Eigen::MatrixXd& Tcov); // This methods attempts to register the point cloud
                                          // versus the map using Tmotion as a first guess

    bool Register(MapTypePtr maptype,
                  Eigen::Affine3d& Tnow,
                  std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                  Eigen::MatrixXd& Tcov); // This methods attempts to register the point cloud
                                          // versus the map using Tmotion as a first guess

    bool Register(MapTypePtr maptype,
                  Eigen::Affine3d& Tnow,
                  pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                  Eigen::MatrixXd& Tcov); // This methods attempts to register the point cloud
                                          // versus the map using Tmotion as a first guess

    std::string ToString();

protected:
    NDTDLRegType(RegParamPtr paramptr);

    NDTMatcherD2DN* matcher3D_;
    NDTMatcherD2D* matcher3D_2_;
    string super_important_parameter_;

    int matcher2D_ITR_MAX_ = 35;
    bool matcher2D_step_control_ = true;
    int matcher2D_n_neighbours_ = 2;
    bool multires_ = false;
    bool SoftConstraints_ = false;
    bool plot_registration_ = true;

    std::vector<double> resolutions_, resolution_local_factors_;

private:
    friend class GraphFactory;
};

class NDTDLRegParam : public registrationParameters
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTDLRegParam();

    void GetParametersFromRos(); // Get parametes from ros e.g. from the ros parameter server

    NDTDLRegParam();

    // Matcher
    int matcher2D_ITR_MAX = 35;
    bool matcher2D_step_control = true;
    int matcher2D_n_neighbours = 2;
    bool multires = false;
    bool SoftConstraints = false;
    std::vector<double> resolutions, resolution_local_factors;

private:
    friend class GraphFactory;
};

} // namespace graph_map
} // namespace perception_oru

#endif // TEMPLATE_REG_TYPE_H
