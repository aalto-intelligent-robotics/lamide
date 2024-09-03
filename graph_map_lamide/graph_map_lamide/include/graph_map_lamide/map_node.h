#ifndef MAP_NODE_H
#define MAP_NODE_H
#include "Eigen/Dense"
#include "boost/serialization/shared_ptr.hpp"
#include "boost/shared_ptr.hpp"
#include "graph_map_lamide/map_type.h"
#include "graphfactory.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/serialization.h"
#include "stdio.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <iostream>
#include <ndt_generic_lamide/point_types.h>

namespace perception_oru
{
namespace graph_map
{

#define NODE_LINK_ID_PREFIX "submap_node"

/*!
 * ... Class to represent a node ...
 */
class Node
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Node();

    // virtual const Eigen::Affine3d& GetPose() const;

    virtual Eigen::Affine3d& GetPose();

    virtual void SetPose(const Eigen::Affine3d& pose)
    {
        pose_ = pose;
    }

    bool operator==(const Node& node_compare);

    virtual unsigned int GetId() const
    {
        return id_;
    }

    virtual std::string ToString()
    {
        return "base node";
    }

    virtual bool WithinRadius(const Affine3d& pose, const double& radius);

    static bool DetectNewNode(unsigned int& node_id,
                              const NodePtr& node); // compare node with node_id, if different,
                                                    // node.GetId is assigned to node_id

protected:
    unsigned int id_;
    Eigen::Affine3d pose_;

private:
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& id_;
        ar& pose_;
    }
};
/*!
 * ... Class to represent a map node ...
 */
class MapNode : public Node
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapNode();

    void updateMap(const Eigen::Affine3d& Tnow,
                   pcl::PointCloud<pcl::PointXYZL>& cloud,
                   bool simple = false,
                   bool cluster = false);

    void updateMap(const Eigen::Affine3d& Tnow,
                   std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                   bool simple = false,
                   bool cluster = false);

    virtual std::string ToString();

    virtual MapTypePtr GetMap()
    {
        if (isOnDisk() || map_ == NULL)
        {
            // FIXME: this is dangerous, but the caller should load maps, to avoid unnecessary "load
            // all maps" -type of things, that will crash the program
            bool success = loadFromDisk();
            if(!success)
            {
                return NULL;
            }
        }
        return map_;
    }

    virtual bool Initialized()
    {
        return initialized_;
    }

    Eigen::Affine3d GetMapPose(bool obs_centroid = false);

    ndt_generic::Affine3dSTLVek GetObservationVector(bool local_frame = false);

    bool WithinRadius(const Affine3d& pose, const double& radius, bool obs_centroid = false);

    void AddPoseFramePair(const Eigen::Affine3d& Tnow, const Eigen::Affine3d& Tref);

    void UpdatePoseFrame();

    std::vector<Eigen::Affine3d> GetPoseEst();

    std::vector<Eigen::Affine3d> GetPoseExt();

    bool isOnDisk()
    {
        return onDisk_;
    }

    bool unloadToDisk(const std::string& mapid = "");

    bool loadFromDisk();

    void setPath(const std::string& path)
    {
        path_ = path;
    }

    void fixPathStrings();

    std::string getUnloadFile() const;

    void setUnloadFile(const std::string& newFile);

protected:
    MapTypePtr map_ = NULL;
    MapNode(const Eigen::Affine3d& pose, const MapParamPtr& mapparam, const std::string& path);
    bool initialized_ = false;
    std::vector<Eigen::Affine3d> poseFrames_;         // submap frame
    std::vector<Eigen::Affine3d> poseFramesExternal_; // world frame
    std::string path_;
private:
    bool onDisk_ = false;
    std::string unloadFile_ = "";
    friend class GraphFactory;
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        ar& boost::serialization::base_object<Node>(*this);
        // ar& map_;
        ar& initialized_;
        ar& poseFrames_;
        ar& poseFramesExternal_;
        ar& unloadFile_;
        ar& onDisk_;
    }
};

std::string GetNodeLinkName(const MapNodePtr& node);
} // namespace graph_map

} // namespace perception_oru
#endif // MAP_NODE_H
