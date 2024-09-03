#ifndef SCAN_MAP_TYPE_H
#define SCAN_MAP_TYPE_H

#include "boost/serialization/export.hpp"
#include "graph_map_lamide/graphfactory.h" //includes the full list of forward declarations
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sstream"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <graph_map_lamide/map_type.h>


#define scan_map_type_name "scan_map"
namespace perception_oru
{
namespace graph_map
{

class ScanMapType : public MapType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~ScanMapType();

    ScanMapType()
    {
    }

    ScanMapType(MapParamPtr paramptr);

    virtual void update(const Eigen::Affine3d& Tsensor,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple = false); // Mandatory, base method implemented as pure virtual

    virtual void update(const Eigen::Affine3d& Tsensor,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple = false); // Mandatory, base method implemented as pure virtual

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

class ScanMapParam : public MapParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~ScanMapParam()
    {
    }

    void GetParametersFromRos();

    string SuperImportantMapParameter;

    ScanMapParam()
    {
    }

protected:
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

#endif // SCAN_MAP_TYPE_H
