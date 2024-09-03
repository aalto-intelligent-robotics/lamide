#ifndef NARF_REG_TYPE_H
#define NARF_REG_TYPE_H

#include "Eigen/Dense"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/narf/narf_map_type.h"
#include "graph_map_lamide/reg_type.h"
#include "graph_map_lamide/template/template_map_type.h"
#include "math.h"
#include "pcl/io/pcd_io.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

#include <pcl/point_cloud.h>


#define narf_reg_type_name "narf_reg"
namespace perception_oru
{
namespace graph_map
{
class NarfRegType : public registrationType
{
public:
    ~NarfRegType();
    bool Register(MapTypePtr maptype,
                  Eigen::Affine3d& Tnow,
                  pcl::PointCloud<pcl::PointXYZL>& cloud,
                  Eigen::MatrixXd& Tcov); // This methods attempts to register the point cloud
                                          // versus the map using Tmotion as a first guess
protected:
    string super_important_parameter_;
    NarfRegType(RegParamPtr paramptr);

private:
    friend class GraphFactory;
};

class NarfRegTypeParam : public registrationParameters
{
public:
    ~NarfRegTypeParam();
    void GetParametersFromRos(); // Get parametes from ros e.g. from the ros parameter server
    // but all your parameters here
    string super_important_parameter_;

protected:
    NarfRegTypeParam(); // Constructor is protected to allow only graphcatory to instanciate or
                        // derived classes create this type
private:
    friend class GraphFactory;
};
} // namespace graph_map

} // namespace perception_oru

#endif // TEMPLATE_REG_TYPE_H
