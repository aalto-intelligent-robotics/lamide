/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef LSL_LAZZY_GRID_HH
#define LSL_LAZZY_GRID_HH

#include <ndt_map_lamide/ndt_cell.h>
#include <boost/serialization/base_object.hpp>
#include "boost/serialization/serialization.hpp"
#include <boost/serialization/export.hpp>
#include "boost/serialization/array.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/export.hpp"
#include "boost/serialization/list.hpp"
#include <boost/serialization/vector.hpp>
#include "stdio.h"
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>
#include <ndt_map_lamide/spatial_index.h>
#include <set>

namespace perception_oru
{

/** \brief A spatial index represented as a grid map
    \details A grid map with delayed allocation of cells.
*/
class LazyGrid : public SpatialIndex
{
public:
    LazyGrid(double cellSize);
    LazyGrid(double cellSizeX_, double cellSizeY_, double cellSizeZ_);
    /// Copy the parameter of the lazygrid but not the content
    LazyGrid(LazyGrid* prot);
    /// Copy the parameter of the lazygrid but not the content
    LazyGrid(const LazyGrid& prot);
    LazyGrid(double sizeXmeters,
             double sizeYmeters,
             double sizeZmeters,
             double cellSizeX,
             double cellSizeY,
             double cellSizeZ,
             double _centerX,
             double _centerY,
             double _centerZ,
             NDTCell* cellPrototype);
    virtual ~LazyGrid();

    virtual NDTCell* getCellForPoint(const pcl::PointXYZL& point);
    virtual NDTCell* addPoint(const pcl::PointXYZL& point);

    // these two don't make much sense...
    /// iterator through all cells in index, points at the begining
    virtual typename SpatialIndex::CellVectorItr begin();
    virtual typename SpatialIndex::CellVectorConstItr begin() const;
    /// iterator through all cells in index, points at the end
    virtual typename SpatialIndex::CellVectorItr end();
    virtual typename SpatialIndex::CellVectorConstItr end() const;
    virtual int size();

    /// clone - create an empty object with same type
    virtual SpatialIndex* clone() const;
    virtual SpatialIndex* copy() const;

    /// method to return all cells within a certain radius from a point
    virtual void getNeighbors(const pcl::PointXYZL& point,
                              const double& radius,
                              std::vector<NDTCell*>& cells);
    /// method to return all cells within a certain radius from a point. Clone the cell once and the
    /// return shared pointers.
    virtual void getNeighborsShared(const pcl::PointXYZL& point,
                                    const double& radius,
                                    std::vector<boost::shared_ptr<NDTCell>>& cells);

    /// sets the cell factory type
    virtual void setCellType(NDTCell* type);

    /**
     * Set the attribute of center
     * @param cx
     * @param cy
     * @param cz
     * @param init if true, if the size and center are set, the NDTCell pointer memory is allocated
     */
    virtual void setCenter(const double& cx, const double& cy, const double& cz, bool init = true);
    /**
     * Set the attribute of size. calculate sizeX = abs(ceil(sizeXmeters/cellSizeX)); so cellSizeX
     * needs to be defined
     * @param cx the size along x in meters
     * @param cy the size along y in meters
     * @param cz the size along z in meters
     * @param init if true, if the size and center are set, the NDTCell pointer memory is allocated
     */
    virtual void setSize(const double& sx, const double& sy, const double& sz, bool init = true);
    bool insertCell(NDTCell cell);

    virtual NDTCell* getClosestNDTCell(const pcl::PointXYZL& pt, bool checkForGaussian = true);
    virtual std::vector<NDTCell*> getClosestNDTCells(const pcl::PointXYZL& pt,
                                                     int& n_neigh,
                                                     bool checkForGaussian = true);
    virtual std::vector<boost::shared_ptr<NDTCell>> getClosestNDTCellsShared(
        const pcl::PointXYZL& pt, int& n_neigh, bool checkForGaussian = true);
    virtual std::vector<NDTCell*> getClosestCells(const pcl::PointXYZL& pt);

    virtual inline void getCellAt(int indX, int indY, int indZ, NDTCell*& cell)
    {
        if (indX < sizeX && indY < sizeY && indZ < sizeZ && indX >= 0 && indY >= 0 && indZ >= 0)
        {
            cell = dataArray[indX][indY][indZ];
        }
        else
        {
            cell = NULL;
        }
    }

    virtual inline void getCellAt(const pcl::PointXYZL& pt, NDTCell*& cell)
    {
        int indX, indY, indZ;
        this->getIndexForPoint(pt, indX, indY, indZ);
        this->getCellAt(indX, indY, indZ, cell);
    }

    /// automatically allocate the cell if needed (checks that the indexes etc. are correct).
    virtual void getCellAtAllocate(const pcl::PointXYZL& pt, NDTCell*& cell);

    // ORU_FIXME: these two are now not needed anymore
    virtual inline void getNDTCellAt(int indX, int indY, int indZ, NDTCell*& cell)
    {
        //	        std::cout << indX << " < " << sizeX << " && " << indY << " < " << sizeY << " &&
        //" << indZ << " < " << sizeZ << " && " << indX << " >= " << 0 << " && " << indY << " >= "
        //<<0 << " && " << indZ << " >= " << 0 << std::endl;
        if (indX < sizeX && indY < sizeY && indZ < sizeZ && indX >= 0 && indY >= 0 && indZ >= 0)
        {
            //				std::cout << "Looking in the array" << std::endl;
            cell = (dataArray[indX][indY][indZ]);
        }
        else
        {
            cell = NULL;
        }
    }

    virtual inline void getNDTCellAt(const pcl::PointXYZL& pt, NDTCell*& cell)
    {
        int indX, indY, indZ;
        this->getIndexForPoint(pt, indX, indY, indZ);
        //std::cout << "Index " << indX << " " << indY << " " << indZ << std::endl;
        this->getNDTCellAt(indX, indY, indZ, cell);
    }

    void getCellSize(double& cx, double& cy, double& cz) const;
    void setCellSize(double cx, double cy, double cz);
    void getGridSize(int& cx, int& cy, int& cz);
    void getGridSizeInMeters(double& cx, double& cy, double& cz) const;
    void setGridSizeInMeters(double& cx, double& cy, double& cz);
    void getCenter(double& cx, double& cy, double& cz) const;
    virtual void getIndexForPoint(const pcl::PointXYZL& pt, int& idx, int& idy, int& idz);
    NDTCell* getProtoType()
    {
        return protoType;
    }

    virtual void initialize();
    virtual void initializeAll();

    NDTCell**** getDataArrayPtr()
    {
        return dataArray;
    }

    /// reads map contents from .jff file
    virtual int loadFromJFF(FILE* jffin);
    bool traceLine(const Eigen::Vector3d& origin,
                   const pcl::PointXYZL& endpoint,
                   const Eigen::Vector3d& diff,
                   const double& maxz,
                   std::vector<NDTCell*>& cells);

    bool traceLine(const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& endpoint,
                   const Eigen::Vector3d& diff,
                   const double& maxz,
                   std::vector<NDTCell*>& cells)
    {
        pcl::PointXYZL ep;
        ep.x = endpoint(0);
        ep.y = endpoint(1);
        ep.z = endpoint(2);
        return traceLine(origin, ep, diff, maxz, cells);
    }

    /**
     * return all cells on a line after initializing them.
     * @param origin origin point of the line
     * @param endpoint not the end point... just return false if maxz is lower than the z of this
     * point...
     * @param diff slope and length of the line. Usually calculated form endpoint. Should be removed
     * from the parameters as it is redundant.
     * @param maxz return false if endpoint.z is higher than this...
     * @param cells the cells on the line
     * @param final_point returns the actual final point of the curve. Useful mainly to know if the
     * origin+diff was outside of the map.
     * @return true in all cases and false if the maxz and endpoint trigger happens...
     */
    bool traceLineWithEndpoint(const Eigen::Vector3d& origin,
                               const Eigen::Vector3d& endpoint,
                               const double& maxz,
                               std::set<NDTCell*>& cells,
                               Eigen::Vector3d& final_point);
    bool isInside(const pcl::PointXYZL& pt)
    {
        int indX, indY, indZ;
        this->getIndexForPoint(pt, indX, indY, indZ);
        return (indX < sizeX && indY < sizeY && indZ < sizeZ && indX >= 0 && indY >= 0 &&
                indZ >= 0);
    }
    void InitializeDefaultValues();
    std::string GetDataString();
    std::string ToString();

    /**
     * Copy the cell into the lazygrid
     * @param cell to copy
     * @return
     */
    NDTCell* copyCell(const pcl::PointXYZL& pt, const NDTCell& cell_to_copy);
    NDTCell* copyCell(const NDTCell& cell_to_copy);

protected:
    bool initialized;
    NDTCell**** dataArray;
    // bool ***linkedCells;
    NDTCell* protoType;
    std::vector<NDTCell*> activeCells;
    bool centerIsSet, sizeIsSet;

    double sizeXmeters, sizeYmeters, sizeZmeters;
    double cellSizeX, cellSizeY, cellSizeZ;
    double centerX, centerY, centerZ;
    int sizeX, sizeY, sizeZ;

    virtual bool checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian = true);

private:
    LazyGrid()
    {
        InitializeDefaultValues();
    }

    friend class boost::serialization::access;
    template <class Archive> void save(Archive& ar, const unsigned int version) const
    {
        ar& boost::serialization::base_object<SpatialIndex>(*this);
        ar& protoType;
        ar& sizeX& sizeY& sizeZ;
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
        ar& activeCells;
        ar& centerIsSet& sizeIsSet& initialized; // Serialize all primitive types
        ar& sizeXmeters& sizeYmeters& sizeZmeters;
        ar& cellSizeX& cellSizeY& cellSizeZ;
        ar& centerX& centerY& centerZ;
    }
    template <class Archive> void load(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<SpatialIndex>(*this);
        ar& protoType;
        ar& sizeX& sizeY& sizeZ;

        dataArray = new NDTCell***[sizeX];
        for (int i = 0; i < sizeX; i++)
        {
            dataArray[i] = new NDTCell**[sizeY];
            for (int j = 0; j < sizeY; j++)
            {
                dataArray[i][j] = new NDTCell*[sizeZ];
                // set all cells to NULL
                memset(dataArray[i][j], 0, sizeZ * sizeof(NDTCell*));
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
        ar& activeCells;
        ar& centerIsSet& sizeIsSet& initialized; // Serialize all primitive types
        ar& sizeXmeters& sizeYmeters& sizeZmeters;
        ar& cellSizeX& cellSizeY& cellSizeZ;
        ar& centerX& centerY& centerZ;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; // namespace perception_oru
//#include<ndt_map/impl/lazy_grid.hpp>

#endif
