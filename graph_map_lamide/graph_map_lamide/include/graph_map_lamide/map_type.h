/**
 *  file map_type.h.
 */
#ifndef MAPTYPE_H
#define MAPTYPE_H
#include "boost/serialization/serialization.hpp"
#include "eigen3/Eigen/Dense"
#include "graphfactory.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/serialization.h"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"

#include <Eigen/Eigen>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <iostream>
#include <ndt_generic_lamide/point_types.h>
#include <pcl/point_cloud.h>
#include <stdio.h>


namespace perception_oru
{
namespace graph_map
{

using std::cerr;
using std::cout;
using std::endl;
using std::string;
/*!
 * ... Abstract class to present parameters for "mapType". this class contain generic parameters
 * forr all map types.  ...
 */

class MapType
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapType();
    /*!
     * \brief update attempts to update the map based on point cloud data and the pose where the
     * scan was taken from(scanner) in the world fram \param Tnow transformation from world to
     * sensor frame \param cloud data to update map with
     */
    bool Initialized() const
    {
        return initialized_;
    }
    /*!
     * \brief updateMap updates the map with could given Tnow iff enable_mapping_
     * \param Tnow the source of cloud
     * \param cloud is the point cloud used to map update the map
     * \param simple - Henrik please comment on this parameter.
     */
    virtual void updateMap(const Eigen::Affine3d& Tnow,
                           pcl::PointCloud<pcl::PointXYZL>& cloud,
                           bool simple = false,
                           bool cluster = false);

    /*!
     * \brief updateMap updates the map with could given Tnow iff enable_mapping_
     * \param Tnow the source of cloud
     * \param cloud is the point cloud used to map update the map - includes additional information
     * about intensity of measurments \param simple - Henrik please describe this parameter.
     */
    virtual void updateMap(const Eigen::Affine3d& Tnow,
                           std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                           bool simple = false,
                           bool cluster = false);

    virtual bool CompoundMapsByRadius(MapTypePtr target,
                                      const Affine3d& T_source,
                                      const Affine3d& T_target,
                                      double radius = 5);

    virtual double Score(MapTypePtr& source, const Affine3d& T)
    {
        return 0;
    } // score for registration

    virtual double Overlap(MapTypePtr& source, const Affine3d& T)
    {
        return 0;
    } // measuring  the overlap

    virtual void PlotScan(pcl::PointCloud<pcl::PointXYZL>& cloud,
                          Eigen::Affine3d& T,
                          const std::string& frame_id)
    {
    }

    virtual void PlotScan(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                          Eigen::Affine3d& T,
                          const std::string& frame_id)
    {
    }

    virtual std::string GetMapName() const
    {
        return mapName_;
    }

    virtual std::string ToString();

    uint64_t GetStamp() const;

    Eigen::Affine3d GetObservationCentroid()
    {
        return mean_obs_location_;
    } // in local_map_frame

    ndt_generic::Affine3dSTLVek GetObservationVector()
    {
        return pose_est_;
    } // in local_map_frame

    virtual void GetCloud(pcl::PointCloud<pcl::PointXYZL>& cloud, bool downsampled = false);

    /*!
     * \brief DownSampleCloud uses the full pointclod which was aquired to create a lightweight
     * pointcloud, stored in downsampled_cloud_
     */
    virtual pcl::PointCloud<pcl::PointXYZL>::Ptr DownSampleCloud()
    {
    }

    MapParamPtr GetPrototype();

    virtual ~MapType()
    {
        prototype_.reset();
    }

    void GetSize(double& sizex, double& sizey, double& sizez)
    {
        sizex = sizex_;
        sizey = sizey_;
        sizez = sizez_;
    }

protected:
    virtual void update(const Eigen::Affine3d& Tnow,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple = false,
                        bool cluster = false) = 0;

    virtual void update(const Eigen::Affine3d& Tnow,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple = false,
                        bool cluster = false) = 0;

    MapType(MapParamPtr param);

    double sizex_ = 0;
    double sizey_ = 0;
    double sizez_ = 0;
    double max_range_ = 130;
    double min_range_ = 0.6;
    bool initialized_ = false;
    bool enable_mapping_ = true;
    bool store_points_ = false;
    uint64_t stamp;
    std::string mapName_ = "";
    ndt_generic::Affine3dSTLVek pose_est_;
    Eigen::Affine3d mean_obs_location_ = Eigen::Affine3d::Identity();
    unsigned int observations_ = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZL>> input_cloud_;
    pcl::PointCloud<pcl::PointXYZL> downsampled_cloud_;
    MapParamPtr prototype_ = NULL;

    // pcl::octree::OctreePointCloudSinglePoint<pcl::PointXYZL> octree;mapParPtr

    /*-----Boost serialization------*/
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& sizex_& sizey_& sizez_;
        ar& max_range_& min_range_;
        ar& initialized_;
        ar& enable_mapping_;
        ar& mapName_;
        ar& mean_obs_location_;
        ar& pose_est_;
        ar& input_cloud_;
        ar& prototype_;
    }
};

class MapParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~MapParam() = 0;

    std::string getMapName() const
    {
        return mapName_;
    }

    virtual void GetParametersFromRos();

    virtual std::string ToString();

    double sizex_ = 150;
    double sizey_ = 150;
    double sizez_ = 12;
    double max_range_ = 130;
    double min_range_ = 0.6;
    bool enable_mapping_ = true;
    bool store_points = false;

    MapParam()
    {
    }

    MapParam(const MapParam& MapParam);

    virtual MapParamPtr clone()
    {
        cerr << "Cloning of map parameters not implemented" << endl;
        return NULL;
    }

protected:
    std::string mapName_ = "";

    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& sizex_;
        ar& sizey_;
        ar& sizez_;
        ar& max_range_;
        ar& min_range_;
    }
};
} // namespace graph_map
} // namespace perception_oru
#endif // MAPTYPE_H
