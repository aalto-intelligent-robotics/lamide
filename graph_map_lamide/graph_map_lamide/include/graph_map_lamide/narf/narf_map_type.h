#ifndef NARF_MAP_TYPE_H
#define NARF_MAP_TYPE_H

#include "graph_map_lamide/graphfactory.h" //includes the full list of forward declarations
#include "mutex"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "thread"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/thread/thread.hpp>
#include <graph_map_lamide/map_type.h>
#include <pcl/console/parse.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>


#define narf_map_name "narf"
namespace perception_oru
{
namespace graph_map
{

class NarfMapType : public MapType
{
public:
    // Mandatory
    ~NarfMapType();
    NarfMapType()
    {
    }
    NarfMapType(MapParamPtr paramptr);

    virtual void update(const Eigen::Affine3d& Tnow,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple = false); // Mandatory, base method implemented as pure virtual

    virtual void update(const Eigen::Affine3d& Tnow,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple = false); // Mandatory, base method implemented as pure virtual

    void NarfViewer();

    // Optional
    bool CompoundMapsByRadius(MapTypePtr target,
                              const Affine3d& T_source,
                              const Affine3d& T_target,
                              double radius); // Optional
    pcl::visualization::PCLVisualizer viewer;
    pcl::RangeImage::CoordinateFrame coordinate_frame;
    pcl::visualization::RangeImageVisualizer range_image_widget;
    std::thread t_viewer;
    bool viewer_updated = false;
    std::mutex m;

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

    void setViewerPose(pcl::visualization::PCLVisualizer& viewer,
                       const Eigen::Affine3f& viewer_pose);
};
class NarfMapParam : public MapParam
{
public:
    ~NarfMapParam()
    {
    }
    NarfMapParam()
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
