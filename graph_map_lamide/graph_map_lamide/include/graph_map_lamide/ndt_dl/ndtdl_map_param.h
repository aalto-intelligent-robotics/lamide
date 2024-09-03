#ifndef NDTDL_MAP_PARAM_H
#define NDTDL_MAP_PARAM_H

#include "graph_map_lamide/map_type.h"
#include "ros/ros.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <string.h>
namespace perception_oru
{
namespace graph_map
{

class NDTDLMapParam : public MapParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTDLMapParam()
    {
    }

    NDTDLMapParam()
    {
    }

    NDTDLMapParam(const NDTDLMapParam& par);

    MapParamPtr clone();

    void GetParametersFromRos();

    std::vector<double> resolutions_;

    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapParam>(*this);
        ar& resolutions_;
    }
};

} // namespace graph_map
} // namespace perception_oru
#endif
