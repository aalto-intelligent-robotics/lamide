#ifndef TEMPLATE_MAP_TYPE_H
#define TEMPLATE_MAP_TYPE_H

#include "graph_map_lamide/graphfactory.h" //includes the full list of forward declarations
#include "ros/node_handle.h"
#include "ros/ros.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <graph_map_lamide/map_type.h>


#define template_map_name "template"
namespace perception_oru
{
namespace graph_map
{

class TemplateMapType : public MapType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~TemplateMapType();

    TemplateMapType()
    {
    }

    TemplateMapType(MapParamPtr paramptr);

    virtual void update(const Eigen::Affine3d& Tsensor,
                        pcl::PointCloud<pcl::PointXYZL>&
                            cloud); // Mandatory, base method implemented as pure virtual

    virtual bool CompoundMapsByRadius(MapTypePtr target,
                                      const Affine3d& T_source,
                                      const Affine3d& T_target,
                                      double radius); // Optional

private:
    friend class GraphFactory; // objects of type <template_map_type> are created by using teh
                               // factory design pattern, don't forget to register
                               // <template_map_type> for creation in factory

    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapType>(*this);
    }
    /*-----End of Boost serialization------*/
};

class TemplateMapParam : public MapParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~TemplateMapParam()
    {
    }

    void GetParametersFromRos();

    string SuperImportantMapParameter;

protected:
    TemplateMapParam();

private:
    friend class GraphFactory;
    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapParam>(*this);
    }
};

} // namespace graph_map
} // namespace perception_oru

#endif // TEMPLATE_MAP_TYPE_H
