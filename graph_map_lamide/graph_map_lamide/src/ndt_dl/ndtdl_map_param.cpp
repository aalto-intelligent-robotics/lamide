#include "graph_map_lamide/ndt_dl/ndtdl_map_param.h"

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTDLMapParam)

namespace perception_oru
{
namespace graph_map
{

void NDTDLMapParam::GetParametersFromRos()
{
    MapParam::GetParametersFromRos();
    ros::NodeHandle nh("~");
    cout << "reading parameters from ros inside NDTDLMapParam::GetRosParametersFromRos()" << endl;

    if (!nh.getParam("resolutions", resolutions_))
    {
        std::cerr << "Failed to read resolutions param(!)" << std::endl;
    }
}

NDTDLMapParam::NDTDLMapParam(const NDTDLMapParam& par) : MapParam(par)
{
    resolutions_ = par.resolutions_;
}

MapParamPtr NDTDLMapParam::clone()
{
    NDTDLMapParamPtr ptr = NDTDLMapParamPtr(new NDTDLMapParam());
    ptr->resolutions_ = resolutions_;
    return ptr;
}

} // namespace graph_map

} // namespace perception_oru
