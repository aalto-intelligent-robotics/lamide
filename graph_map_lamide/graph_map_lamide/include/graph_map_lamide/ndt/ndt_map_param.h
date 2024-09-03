#ifndef NDT2DMAPPARAM_H
#define NDT2DMAPPARAM_H
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

/*!
 * ... Parameter class for mapType. Class is by choice of design fully public.  ...
 */

class NDTMapParam : public MapParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTMapParam()
    {
    }

    NDTMapParam()
    {
    }

    NDTMapParam(const NDTMapParam& par);

    MapParamPtr clone();

    void GetParametersFromRos();

    double resolution_ = 0.4;
    bool match2D_ = false;
    double positive_update_static_ = 0.6;
    double negative_update_static_ = 0.4;
    double positive_update_dynamic_ = 0.6;
    double negative_update_dynamic_ = 0.4;
    double eta_static_ = 0.2;
    double eta_dynamic_ = 0.2;
    double w_own_ = 1.0;
    double w_cluster_ = 1.0;
    double max_occupancy_ = 255;
    double sensor_noise_ = 0.06;
    int maxNumPoints_ = 1e5;

    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapParam>(*this);
        ar& resolution_;
        ar& match2D_;
        ar& positive_update_static_;
        ar& negative_update_static_;
        ar& positive_update_dynamic_;
        ar& negative_update_dynamic_;
        ar& eta_static_;
        ar& eta_dynamic_;
        ar& w_own_;
        ar& w_cluster_;
        ar& max_occupancy_;
        ar& sensor_noise_;
        ar& maxNumPoints_;
    }
};

} // namespace graph_map
} // namespace perception_oru
#endif // NDT2DMAPPARAM_H
