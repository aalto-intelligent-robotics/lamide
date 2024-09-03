#ifndef SCAN_REG_TYPE_H
#define SCAN_REG_TYPE_H

#include "Eigen/Dense"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/reg_type.h"
#include "graph_map_lamide/scanmap/scan_map_type.h"
#include "math.h"
#include "pcl/io/pcd_io.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

#include <pcl/point_cloud.h>


// BOOST_CLASS_EXPORT(perception_oru::graph_map::ScanRegParam)
// BOOST_CLASS_EXPORT(perception_oru::graph_map::ScanRegType)

#define scan_reg_type_name "scan_reg_type"
namespace perception_oru
{
namespace graph_map
{
class ScanRegType : public registrationType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~ScanRegType()
    {
    }
    // ScanRegType():registrationType(){};
protected:
    bool Register(MapTypePtr maptype,
                  Eigen::Affine3d& Tnow,
                  pcl::PointCloud<pcl::PointXYZL>& cloud,
                  Eigen::MatrixXd& Tcov)
    {
        std::cerr << "No registration for pcl::PointXYZL implemented" << endl;
    } // This methods attempts to register the point cloud versus the map using the affine
      // transformation guess "Tm"

    bool Register(MapTypePtr maptype,
                  Eigen::Affine3d& Tnow,
                  std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                  Eigen::MatrixXd& Tcov)
    {
        std::cerr << "No registration for std::vector<pcl::PointXYZL> implemented" << endl;
    }

    bool RegisterMap2Map(MapTypePtr target,
                         MapTypePtr source,
                         Eigen::Affine3d& Tdiff,
                         double& match_score)
    {
    }

    ScanRegType(RegParamPtr paramptr);

protected:
    string super_important_parameter_;

    friend class GraphFactory;
};

class ScanRegParam : public registrationParameters
{

public:
    ScanRegParam();

    ~ScanRegParam();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void GetParametersFromRos(); // Get parametes from ros e.g. from the ros parameter server
    // but all your parameters here
    string super_important_parameter_;

protected:
    friend class GraphFactory;
    // Constructor is protected to allow only graphcatory to instanciate or derived classes create
    // this type
};
} // namespace graph_map

} // namespace perception_oru

#endif // TEMPLATE_REG_TYPE_H
