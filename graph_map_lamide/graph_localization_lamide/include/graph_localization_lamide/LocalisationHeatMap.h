#ifndef MAPPED_REGION_H
#define MAPPED_REGION_H
#include "Eigen/Dense"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/array.hpp"
#include "boost/serialization/map.hpp"
#include "boost/serialization/serialization.hpp"
#include "boost/serialization/vector.hpp"
#include "graph_map_lamide/graph_map_navigator.h"
#include "ndt_generic_lamide/eigen_utils.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using std::cerr;
using std::cout;
using std::endl;

namespace pogm = perception_oru::graph_map;

namespace HeatMap
{

class Voxel
{
public:
    Voxel()
    {
    }
    float val = 0;
    static Voxel* CreateVoxel()
    {
        return new Voxel();
    }

private:
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(
        Archive& ar,
        const unsigned int version) // In order to clal this you need to register it to boost using
                                    // "ar.template register_type<LazyGrid>();"
    {
        ar& val;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::pair<Voxel*, Eigen::Vector3i> IndexedVoxel; // contains metadata about the voxel

typedef std::vector<IndexedVoxel> VoxelVector;

class LocalisationHeatMap
{

public:
    LocalisationHeatMap(double resolution,
                        const double map_size_x,
                        const double map_size_y,
                        const double map_size_z);

    LocalisationHeatMap()
    {
    }

    float GetScore(const Eigen::Vector3d& p);

    double DistanceP2VCenter(const Eigen::Vector3d& p, const IndexedVoxel& voxel);

    void PrintGrid();

    bool getNeighborsByRadius(const double& radius,
                              const Eigen::Vector3d& p,
                              VoxelVector& nearby_voxels,
                              bool OnlyInitialized = true);

    Eigen::Vector3d GetVoxelCenter(const Eigen::Vector3i& idx);

    Eigen::Vector3d GetVoxelCenter(const IndexedVoxel& voxel);

    bool getIndexForPoint(const Eigen::Vector3d& p, Eigen::Vector3i& idx);

    void UpdateHeatmapNormal(const Eigen::Vector3d& Tupdsrc, double sigma_update = 10);

    void InitializeNeighborsByRadius(const double& radius, const Eigen::Vector3d& p);

    void InitializeNeighborsByRadius(const double& radius,
                                     const Eigen::Vector3d& p,
                                     VoxelVector& nearby_voxels);

    int GetUpdates()
    {
        return n_updates_;
    }

    static void VoxelFilter(VoxelVector& vek, bool remove_initialized_voxels = false);

private:
    double sizeXmeters, sizeYmeters, sizeZmeters;
    double resolution_;
    int sizeX, sizeY, sizeZ;
    double half_voxel;
    double half_sizeXmeters, half_sizeYmeters, half_sizeZmeters;
    Voxel**** dataArray;
    int n_updates_ = 0;
    double limit_ = 15.0;

    friend class boost::serialization::access;
    template <class Archive> void save(Archive& ar, const unsigned int version) const
    {
        ar& sizeXmeters;
        ar& sizeYmeters;
        ar& sizeZmeters;
        ar& resolution_;
        ar& sizeX;
        ar& sizeY;
        ar& sizeZ;
        ar& half_voxel;
        ar& half_sizeXmeters;
        ar& half_sizeYmeters;
        ar& half_sizeZmeters;
        ar& n_updates_;
        ar& limit_;

        for (int i = 0; i < sizeX; i++)
        {
            for (int j = 0; j < sizeY; j++)
            {
                for (int k = 0; k < sizeZ; k++)
                {
                    ar& dataArray[i][j][k];
                }
            }
        }
    }

    friend class boost::serialization::access;
    template <class Archive> void load(Archive& ar, const unsigned int version)
    {
        ar& sizeXmeters;
        ar& sizeYmeters;
        ar& sizeZmeters;
        ar& resolution_;
        ar& sizeX;
        ar& sizeY;
        ar& sizeZ;
        ar& half_voxel;
        ar& half_sizeXmeters;
        ar& half_sizeYmeters;
        ar& half_sizeZmeters;
        ar& n_updates_;
        ar& limit_;
        dataArray = new Voxel***[sizeX];
        for (int i = 0; i < sizeX; i++)
        {
            dataArray[i] = new Voxel**[sizeY];
            for (int j = 0; j < sizeY; j++)
            {
                dataArray[i][j] = new Voxel*[sizeZ];
                memset(dataArray[i][j], 0, sizeZ * sizeof(Voxel*));
            }
        }

        for (int i = 0; i < sizeX; i++)
        {
            for (int j = 0; j < sizeY; j++)
            {
                for (int k = 0; k < sizeZ; k++)
                {
                    ar& dataArray[i][j][k];
                }
            }
        }
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER()

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class HeatMapInterface
{
public:
    HeatMapInterface(pogm::GraphMapNavigatorPtr graph_map, const double resolution);

    HeatMapInterface()
    {
    }

    void UpdateHeatMaps();

    float GetMapForPoint(const Eigen::Vector3d& p,
                         pogm::MapNodePtr& map_ptr,
                         pogm::GraphMapNavigatorPtr graph_map);

    bool SetAndVerifyInterface(pogm::GraphMapNavigatorPtr& graph);

    std::string ToString();

    static bool LoadHeatMap(const std::string& file_name, boost::shared_ptr<HeatMapInterface>& ptr);

    static void SaveHeatMap(const std::string& file_name, boost::shared_ptr<HeatMapInterface>& ptr);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    double res_ = 0.3;
    double sigma1_ = 0.75, sigma2_ = 2.0;
    bool initialized_ = false;

    pogm::GraphMapNavigatorPtr graph_map_;
    std::vector<std::pair<LocalisationHeatMap*, int>>
        heatmaps_; // contain the local Heatmap and the associated reference frame

    friend class boost::serialization::access;
    template <class Archive> void save(Archive& ar, const unsigned int version) const
    { // In order to clal this you need to register it to boost using "ar.template
      // register_type<LazyGrid>();"

        ar& res_;
        ar& sigma1_;
        ar& sigma2_;
        ar& heatmaps_;
    }
    friend class boost::serialization::access;
    template <class Archive>
    void load(Archive& ar,
              const unsigned int version) // In order to clal this you need to register it to boost
                                          // using "ar.template register_type<LazyGrid>();"
    {
        ar& res_;
        ar& sigma1_;
        ar& sigma2_;
        ar& heatmaps_;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
};

} // namespace HeatMap
#endif // MAPPED_REGION_H
