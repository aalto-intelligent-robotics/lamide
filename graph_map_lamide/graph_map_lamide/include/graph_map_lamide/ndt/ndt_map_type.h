#ifndef NDT2DMAP_TYPE_H
#define NDT2DMAP_TYPE_H
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/visualization/graph_plot.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <graph_map_lamide/ndt/ndt_map_param.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/ndt_map_hmt.h>
#include <ndt_map_lamide/pointcloud_utils.h>
//#include <ndt_fuser_lamide/motion_model_2d.h>
#include "boost/serialization/export.hpp"
#include "eigen_conversions/eigen_msg.h"
#include "pcl_ros/point_cloud.h"
#include "sstream"
#include "stdio.h"

#include <graph_map_lamide/map_type.h>


#define ndt_map_type_name "ndt_map"
namespace perception_oru
{
namespace graph_map
{

class NDTMapType : public MapType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~NDTMapType() override;

    virtual void update(const Eigen::Affine3d& Tsensor,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple = false,
                        bool cluster = false);

    virtual void update(const Eigen::Affine3d& Tsensor,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple = false,
                        bool cluster = false);

    virtual NDTMap* GetNDTMap()
    {
        return map_;
    }

    virtual void GetCloud(pcl::PointCloud<pcl::PointXYZL>& cloud, bool downsampled = false);

    virtual bool CompoundMapsByRadius(MapTypePtr target,
                                      const Affine3d& T_source,
                                      const Affine3d& T_target,
                                      double radius);

    std::string ToString();

    double GetResolution() const
    {
        return resolution_;
    }

    double Score(MapTypePtr& source, const Affine3d& T);

    double Overlap(MapTypePtr& source, const Affine3d& T);

    void PlotScan(pcl::PointCloud<pcl::PointXYZL>& cloud,
                  Eigen::Affine3d& T,
                  const std::string& frame_id);

    pcl::PointCloud<pcl::PointXYZL>::Ptr DownSampleCloud();

    NDTMapType(MapParamPtr paramptr);

    NDTMapType()
    {
    }

protected:
    NDTMap* map_ = NULL;
    double resolution_ = 0.4;
    double resolution_local_factor_ = 1.;
    bool ndt_cell_cloud_ = false;
    bool ndt_filtered_cloud_ = false;
    ros::Publisher cloud_pub, ndt_pub;
    double max_occupancy_ = 255;
    double sensor_noise_ = 0.06;
    int maxNumPoints_ = 1e5;

    friend class GraphFactory;

    void InitializeMap(const Eigen::Affine3d& Td,
                       pcl::PointCloud<pcl::PointXYZL>& cloud,
                       bool simple = false,
                       bool cluster = false);

    void NDTfilterPointcloud(pcl::PointCloud<pcl::PointXYZL>& cloud, double sigma_distance = 100);

    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<MapType>(*this);
        ar& map_;
        ar& resolution_;
        ar& resolution_local_factor_;
    }
    /*-----End of Boost serialization------*/
};

} // namespace graph_map
} // namespace perception_oru
#endif // NDTMAP_TYPE_H
