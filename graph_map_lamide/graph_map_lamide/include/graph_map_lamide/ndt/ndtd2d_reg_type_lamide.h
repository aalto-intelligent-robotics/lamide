#ifndef NDTD2DREGTYPE_LAMIDE_H
#define NDTD2DREGTYPE_LAMIDE_H
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
#include <ndt_registration_lamide/ndt_matcher_d2d_lamide.h>
#include <ndt_registration_lamide/ndt_matcher_d2d_sc.h>
#include "ndt_registration_lamide/ndt_matcher_d2d_sc_2d.h"
#include "graph_map_lamide/ndt/ndt_map_type.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "ros/publisher.h"
#include "ros/node_handle.h"
#include "pcl_ros/point_cloud.h"

#define ndt_lamide_reg_type_name "ndt_d2d_reg_lamide"

namespace perception_oru
{
namespace graph_map
{

class NDTLamideRegType : public registrationType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTLamideRegType();

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

    bool RegisterMap2Map(MapTypePtr map_prev,
                         MapTypePtr map_next,
                         Eigen::Affine3d& Tdiff,
                         double& match_score);

    bool Register(pcl::PointCloud<pcl::PointXYZL>& target,
                  Eigen::Affine3d& Tnow,
                  pcl::PointCloud<pcl::PointXYZL>& src,
                  Eigen::MatrixXd& Tcov);

    std::string ToString();

    void SetLocalResolutionFactor(double resolution)
    {
        resolutionLocalFactor_ = resolution;
    }

    NDTMap* GetRegisteredMap() const
    {
        return ndlocal;
    }

protected:
    NDTLamideRegType(RegParamPtr paramptr);

    NDTMatcherD2D_2D* matcher2D_;

    NDTMatcherLamide* matcher3D_;
    NDTMap* ndlocal = NULL;
    double resolution_ = 0.8, resolutionLocalFactor_ = 1.0;
    int matcher2D_ITR_MAX_ = 35;
    bool matcher2D_step_control_ = true;
    int matcher2D_n_neighbours_ = 2;
    bool multires_ = false;
    bool SoftConstraints_ = false;
    bool plot_registration_ = true;

private:
    friend class GraphFactory;
};

class NDTLamideRegParam : public registrationParameters
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTLamideRegParam();

    void GetParametersFromRos();

    NDTLamideRegParam();

    double resolution = 0.8, resolution_local_factor = 1.0;
    // Matcher
    int matcher2D_ITR_MAX = 35;
    bool matcher2D_step_control = true;
    int matcher2D_n_neighbours = 2;
    bool multires = false;
    bool SoftConstraints = false;

    const std::vector<int> all_labels_ = {0,  1,  10,  11,  13,  15,  16,  18,  20,  30, 31, 32,
                                          40, 44, 48,  49,  50,  51,  52,  60,  70,  71, 72, 80,
                                          81, 99, 252, 253, 254, 255, 256, 257, 258, 259};
    const std::vector<int> static_labels_ = {40, 44, 48, 49, 50, 51, 52,
                                             60, 70, 71, 72, 80, 81, 99};
    const std::vector<int> semistatic_labels_ = {
        10, 11, 13, 15, 16, 18, 20, 30, 31, 32,
    };
    const std::vector<int> dynamic_labels_ = {252, 253, 254, 255, 256, 257, 258, 259};
    const std::vector<int> nonstatic_labels_ = {0,  1,  10,  11,  13,  15,  16,  18,  20,  30,
                                                31, 32, 252, 253, 254, 255, 256, 257, 258, 259};
    const std::vector<int> unknown_labels_ = {0, 1};

protected:


private:
    friend class GraphFactory;
};
} // namespace graph_map
} // namespace perception_oru

#endif // NDTLamideREGTYPE_H
