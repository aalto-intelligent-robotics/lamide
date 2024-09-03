#include "graph_map_lamide/ndt/ndt_map_param.h"

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTMapParam)

namespace perception_oru
{
namespace graph_map
{
using namespace std;

void NDTMapParam::GetParametersFromRos()
{
    MapParam::GetParametersFromRos();
    ros::NodeHandle nh("~");
    cout << "reading parameters from ros inside GetRosParamNDT2D" << endl;
    nh.param("resolution", resolution_, 1.0);
    nh.param("positive_update_static", positive_update_static_, 0.6);
    nh.param("negative_update_static", negative_update_static_, 0.4);
    nh.param("eta_static", eta_static_, 0.2);
    nh.param("positive_update_dynamic", positive_update_dynamic_, 0.7);
    nh.param("negative_update_dynamic", negative_update_dynamic_, 0.3);
    nh.param("eta_dynamic", eta_dynamic_, 0.8);
    nh.param("w_own", w_own_, 1.0);
    nh.param("w_cluster", w_cluster_, 1.0);
    nh.param("max_occupancy", max_occupancy_, 255.0);
    nh.param("sensor_noise", sensor_noise_, 0.06);
    nh.param("maxNumPoints", maxNumPoints_, 100000);
    cout << "resolution: " << resolution_ << endl;
    // nh.param("laser_variance_z",varz,resolution/4);
}
NDTMapParam::NDTMapParam(const NDTMapParam& par) : MapParam(par)
{
    resolution_ = par.resolution_;
    match2D_ = par.match2D_;
    positive_update_static_ = par.positive_update_static_;
    negative_update_static_ = par.negative_update_static_;
    eta_static_ = par.eta_static_;
    positive_update_dynamic_ = par.positive_update_dynamic_;
    negative_update_dynamic_ = par.negative_update_dynamic_;
    eta_dynamic_ = par.eta_dynamic_;
    w_own_ = par.w_own_;
    w_cluster_ = par.w_cluster_;
    max_occupancy_ = par.max_occupancy_;
    sensor_noise_ = par.sensor_noise_;
    maxNumPoints_ = par.maxNumPoints_;
}

MapParamPtr NDTMapParam::clone()
{
    NDTMapParamPtr ptr = NDTMapParamPtr(new NDTMapParam());
    ptr->enable_mapping_ = enable_mapping_;
    ptr->max_range_ = max_range_;
    ptr->min_range_ = min_range_;
    ptr->sizex_ = sizex_;
    ptr->sizey_ = sizey_;
    ptr->sizez_ = sizez_;
    ptr->resolution_ = resolution_;
    ptr->match2D_ = match2D_;
    ptr->positive_update_static_ = positive_update_static_;
    ptr->negative_update_static_ = negative_update_static_;
    ptr->eta_static_ = eta_static_;
    ptr->positive_update_dynamic_ = positive_update_dynamic_;
    ptr->negative_update_dynamic_ = negative_update_dynamic_;
    ptr->eta_dynamic_ = eta_dynamic_;
    ptr->w_own_ = w_own_;
    ptr->w_cluster_ = w_cluster_;
    ptr->max_occupancy_ = max_occupancy_;
    ptr->sensor_noise_ = sensor_noise_;
    ptr->maxNumPoints_ = maxNumPoints_;
    return ptr;
}

} // namespace graph_map
} // namespace perception_oru
