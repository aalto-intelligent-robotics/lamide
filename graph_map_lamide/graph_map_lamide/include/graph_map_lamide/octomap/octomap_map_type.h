#ifndef OCTOMAP_MAP_TYPE_H
#define OCTOMAP_MAP_TYPE_H

#include "graph_map_lamide/graphfactory.h" //includes the full list of forward declarations
#include "mutex"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "thread"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <graph_map_lamide/map_type.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
//#include "octomap/octomap.h"
#include "pcl/octree/octree.h"
#include "pcl/octree/octree_impl.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"



#include <octomap/octomap.h>
namespace perception_oru
{
namespace graph_map
{
#define octomap_map_name "octomap"
class OctomapMapType : public MapType
{
public:
    // Mandatory
    ~OctomapMapType();
    OctomapMapType()
    {
    }
    OctomapMapType(MapParamPtr paramptr);

    virtual void update(const Eigen::Affine3d& Tnow,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple = false); // Mandatory, base method implemented as pure virtual

    virtual void update(const Eigen::Affine3d& Tnow,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple = false); // Mandatory, base method implemented as pure virtual

    bool CompoundMapsByRadius(MapTypePtr target,
                              const Affine3d& T_source,
                              const Affine3d& T_target,
                              double radius); // Optional

    void GetPointcloud(pcl::PointCloud<pcl::PointXYZL>& cloud);

    octomap::OcTree* GetTree() const
    {
        return tree_;
    }
    // pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZL> *occ;
protected:
    octomap::OcTree* tree_;

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
class OctomapMapTypeParam : public MapParam
{
public:
    ~OctomapMapTypeParam()
    {
    }
    OctomapMapTypeParam()
    {
    }
    void GetParametersFromRos();
    string SuperImportantMapParameter;

private:
    friend class GraphFactory;
    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapParam>(*this);
    }
    /*-----End of Boost serialization------*/
};

} // namespace graph_map
} // namespace perception_oru

#endif // TEMPLATE_MAP_TYPE_H
