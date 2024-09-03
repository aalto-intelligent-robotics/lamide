#ifndef GRAPH_MAP_H
#define GRAPH_MAP_H
#include "Eigen/Dense"
#include "boost/serialization/serialization.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/shared_ptr.hpp"
#include "graph_map_lamide/factor.h"
#include "graph_map_lamide/map_node.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "graph_map_lamide/voxelgrid.h"
#include "graphfactory.h"
#include "mutex"
#include "ndt/ndt_map_type.h"
#include "ndt_map_lamide/ndt_map.h"
#include "pcl/io/ply_io.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "stdio.h"

#include <Eigen/StdVector>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <iostream>
#include <octomap/octomap.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <stdint.h>

namespace perception_oru
{
namespace graph_map
{

struct DynamicStatistic
{
    unsigned int dynamic_ = 0;
    unsigned int semistatic_ = 0;
    unsigned int static_ = 0;
    unsigned int unknown_ = 0;

    unsigned int sum()
    {
        return dynamic_ + semistatic_ + static_ + unknown_;
    }

    std::string serialize()
    {
        std::stringstream ss;
        ss << dynamic_ << ", " << semistatic_ << ", " << static_ << ", "
           << unknown_;
        return ss.str();
    }
};
class DynamicStatistics
{
public:
    void add(DynamicStatistic stat)
    {
        stats_.push_back(stat);
    }

    DynamicStatistic operator[](unsigned int i)
    {
        if(i < stats_.size())
        {
            return stats_[i];
        }
    }

    unsigned int size()
    {
        return stats_.size();
    }

    std::string serialize()
    {
        return sum_.serialize();
    }

    void count()
    {
        for(unsigned int i = 0; i < stats_.size(); i++)
        {
            sum_.dynamic_ += stats_[i].dynamic_;
            sum_.semistatic_ += stats_[i].semistatic_;
            sum_.static_ += stats_[i].static_;
            sum_.unknown_ += stats_[i].unknown_;
        }
        unsigned int tot = sum_.sum();
        p_dynamic_ = (double)sum_.dynamic_ / (double)tot;
        p_semistatic_ = (double)sum_.semistatic_ / (double)tot;
        p_static_ = (double)sum_.static_ / (double)tot;
        p_unknown_ = (double)sum_.unknown_ / (double)tot;
    }

    unsigned int getDynamic()
    {
        return sum_.dynamic_;
    }

    unsigned int getSemistatic()
    {
        return sum_.semistatic_;
    }

    unsigned int getStatic()
    {
        return sum_.static_;
    }

    unsigned int getUnknown()
    {
        return sum_.static_;
    }

    unsigned int getPDynamic()
    {
        return p_dynamic_;
    }

    unsigned int getPSemistatic()
    {
        return p_semistatic_;
    }

    unsigned int getPStatic()
    {
        return p_static_;
    }

    unsigned int getPUnknown()
    {
        return p_static_;
    }

private:
    std::vector<DynamicStatistic> stats_;
    DynamicStatistic sum_;
    double p_dynamic_ = -1;
    double p_static_ = -1;
    double p_semistatic_ = -1;
    double p_unknown_ = -1;
};

typedef std::vector<MapNodePtr>::iterator mapNodeItr;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> EigenAffineVector;
typedef std::vector<NodePtr, Eigen::aligned_allocator<NodePtr>> NodePtrVector;
typedef std::vector<MapNodePtr, Eigen::aligned_allocator<MapNodePtr>> MapNodePtrVector;

class GraphMap
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphMap();

    GraphMap(const GraphMapParamPtr graphparam);

    GraphMap(const Eigen::Affine3d& nodepose,
             const MapParamPtr& mapparam,
             const GraphMapParamPtr graphparam);

    virtual void AddMapNode(const Eigen::Affine3d& diff, const Matrix6d& cov = unit_covar);

    void AddMapNodeExplicitPar(MapParamPtr par_ptr,
                               const Eigen::Affine3d& Tnode,
                               const Matrix6d& cov = unit_covar);

    void AddFixedConstraint(MapNodePtr mapnode,
                            const Eigen::Affine3d& Tdiff,
                            const Matrix6d& cov = unit_covar);

    virtual void RemoveNode(NodePtr node);

    void AddFactor(MapNodePtr prev, MapNodePtr next, const Affine3d& Tdiff, const Matrix6d& cov);

    void UpdateLink(MapNodePtr prev,
                    MapNodePtr next,
                    const Affine3d& Tdiff,
                    const Matrix6d& cov = unit_covar);

    FactorPtr FindLink(MapNodePtr prev, MapNodePtr next);

    void UpdateAllNodePoses();

    MapNodePtr GetCurrentNode();

    MapTypePtr GetCurrentMapNode();

    MapNodePtr GetPreviousNode();

    uint32_t Size() const
    {
        return nodes_.size();
    }

    MapNodePtr GetNode(unsigned int node_id); // as occuring in the node list

    int GetNodeId(const MapNodePtr& node) const;

    const Eigen::Affine3d& GetNodePose(int nodeNr) const;

    Eigen::Affine3d GetCurrentNodePose(bool use_obs_centroid = false);

    Eigen::Affine3d GetPreviousNodePose(bool use_obs_centroid = false);

    virtual std::string ToString();

    mapNodeItr begin()
    {
        return map_nodes_.begin();
    } /* e.g. for(mapNodeItr itr=graph_map->begin(); itr!=graph_map->end();itr++) */

    mapNodeItr end()
    {
        return map_nodes_.end();
    }

    //!
    //! \brief GetCloud Get the pointcloud which was used to update all map nodes
    //! \param cloud the point cloud
    //! \param offset transformation of all points
    //!
    void GetCloud(pcl::PointCloud<pcl::PointXYZL>& cloud, bool downsampled = false);

    void SetActiveNodes(MapNodePtr& prev, MapNodePtr& current)
    {
        prevNode_ = prev;
        currentNode_ = current;
    } // should ONLY be used after the map has been created as the pointers are otherwise set
      // manually

    void UpdatePoseFrame();

    NodePtrVector GetNodes()
    {
        return nodes_;
    }

    const NodePtrVector& GetNodes() const
    {
        return nodes_;
    }

    std::vector<MapNodePtr> GetMapNodes()
    {
        return map_nodes_;
    }

    const std::vector<MapNodePtr>& GetMapNodes() const
    {
        return map_nodes_;
    }

    std::vector<FactorPtr> GetFactors()
    {
        return factors_;
    }

    const std::vector<FactorPtr>& GetFactors() const
    {
        return factors_;
    }

    void setMapUnloading(bool unload)
    {
        map_unloading_ = unload;
    }

    void setMapDir(const std::string& path)
    {
        map_dir_ = path;
    }

    void setMapDirToAll(const std::string& path);

    void setMapFileID();

    const std::string getMapFileID()
    {
        return map_file_id_;
    }

    void ForceSetCurrentNode(MapNodePtr newNode)
    {
        currentNode_ = newNode;
    }

    void ForceSetPreviousNode(MapNodePtr newNode)
    {
        prevNode_ = newNode;
    }

    void RemoveNonStaticVoxels();

    void ClusterMap(bool viewOnly = false,
                    bool clusterSemiStatic = true,
                    bool clusterStatic = true);

    DynamicStatistics GetDynamicStatistics();

    DynamicStatistic GetSigleMapDynamicStatistics(MapNodePtr node = boost::shared_ptr<MapNode>());

public:
    std::mutex m_graph;

protected:
    // The current node
    MapNodePtr currentNode_ = NULL;
    MapNodePtr prevNode_ = NULL;

    // Vector of all nodes in graph
    NodePtrVector nodes_;
    std::vector<FactorPtr> factors_;
    NodePtrVector fixed_nodes_;
    std::vector<MapNodePtr> map_nodes_;
    MapParamPtr mapparam_ = NULL;
    bool identity_orientation_ = false;
    bool map_unloading_ = true;
    std::string map_dir_;
    std::string map_file_id_;

private:
    friend class GraphFactory;
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& currentNode_;
        ar& prevNode_;
        ar& nodes_;
        ar& map_nodes_;
        ar& factors_;
        ar& map_dir_;
        ar& map_file_id_;
    }
};

void SaveGraphMapPCD(const std::string& path, const std::string& filename, GraphMapPtr graph_map);

class GraphMapParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void GetParametersFromRos();
    unsigned int max_size = 0;
    GraphMapParam();
    bool identity_orientation = false;
    virtual ~GraphMapParam()
    {
    }

private:
    friend class GraphFactory;
};
} // namespace graph_map
} // namespace perception_oru
#endif // GRAPH_H
