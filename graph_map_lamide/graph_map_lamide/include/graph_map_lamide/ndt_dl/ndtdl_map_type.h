#ifndef NDTDL_MAP_TYPE_H
#define NDTDL_MAP_TYPE_H
#include "eigen_conversions/eigen_msg.h"
#include "graph_map_lamide/graphfactory.h" //includes the full list of forward declarations
#include "graph_map_lamide/visualization/graph_plot.h"
#include "sstream"
#include "stdio.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <graph_map_lamide/map_type.h>
#include <graph_map_lamide/ndt_dl/ndtdl_map_param.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/ndt_map_hmt.h>
#include <ndt_map_lamide/pointcloud_utils.h>


#define ndtdl_map_type_name "ndt_dl_map"
namespace perception_oru
{
namespace graph_map
{
using std::cerr;
using std::cout;
using std::endl;

class NDTDLMapType : public MapType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTDLMapType();
    virtual void update(const Eigen::Affine3d& Tsensor,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple = false,
                        bool cluster = false); // Mandatory, base method implemented as pure virtual
    virtual void update(const Eigen::Affine3d& Tsensor,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple = false,
                        bool cluster = false); // Mandatory, base method implemented as pure virtual
    // Optional
    virtual bool CompoundMapsByRadius(MapTypePtr target,
                                      const Affine3d& T_source,
                                      const Affine3d& T_target,
                                      double radius); // Optional
    double GetResolutionFlat() const
    {
        return resolutions_[0];
    }
    std::vector<double> GetResolutions() const
    {
        return resolutions_;
    }

    NDTDLMapType(MapParamPtr paramptr);
    NDTDLMapType()
    {
    }

    std::vector<NDTMap*> GetNDTMaps()
    {
        return maps_;
    }
    NDTMap* GetNDTMapFlat()
    {
        return maps_[0];
    }
    std::vector<NDTMap*> maps_;

protected:
    std::vector<double> resolutions_;

    friend class GraphFactory; // objects of type <template_map_type> are created by using teh
                               // factory design pattern, don't forget to register
                               // <template_map_type> for creation in factory
    void InitializeMap(const Eigen::Affine3d& Tsensor,
                       std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds);

    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapType>(*this);
        ar& maps_;
        ar& resolutions_;
    }

    /*-----End of Boost serialization------*/
};

} // namespace graph_map
} // namespace perception_oru

#endif // NDTDL_MAP_TYPE_H
