#ifndef MARKER_VIS_H
#define MARKER_VIS_H
#include "eigen_conversions/eigen_msg.h"
#include "graph_map_lamide/visualization/color_ops.h"
#include "ros/node_handle.h"
#include "string.h"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
#include "visualization_msgs/MarkerArray.h"
namespace perception_oru
{
namespace graph_map
{

class marker_vis
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    marker_vis(const std::string& topic = "/clustered_trajecctory");

    void PlotLabeledPoses(
        const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& poses,
        const std::vector<unsigned int>& idx,
        double size = 0.5);

    std_msgs::ColorRGBA GetColor(double h, double min = 0.3, double max = 1.0);

    // void PlotMatrix(Eigen::MatrixXd &m);

private:
    visualization_msgs::Marker GetDefaultMarker();

    ros::NodeHandle nh_;
    ros::Publisher pub, im_pub;
};

} // namespace graph_map
} // namespace perception_oru
#endif // MARKER_VIS_H
