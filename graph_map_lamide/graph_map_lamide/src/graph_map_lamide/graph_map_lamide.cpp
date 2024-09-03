#include "graph_map_lamide/graph_map.h"

using std::cerr;
using std::cout;
using std::endl;
namespace perception_oru
{
namespace graph_map
{

GraphMap::GraphMap(const Affine3d& nodepose,
                   const MapParamPtr& mapparam,
                   const GraphMapParamPtr graphparam)
{
    Eigen::Affine3d Tinit = Eigen::Affine3d::Identity();
    identity_orientation_ = graphparam->identity_orientation;

    if (identity_orientation_)
    {
        Tinit.translation() = nodepose.translation();
    }
    else
    {
        Tinit = nodepose;
    }

    factors_.clear();
    prevNode_ = NULL;
    // The first node to be added
    currentNode_ = GraphFactory::CreateMapNode(Tinit, mapparam, map_dir_);
    nodes_.push_back(currentNode_);
    map_nodes_.push_back(currentNode_);
    mapparam_ = mapparam;
    setMapFileID();
}

GraphMap::GraphMap(const GraphMapParamPtr graphparam)
{
    GraphMap();
    identity_orientation_ = graphparam->identity_orientation;
}

GraphMap::GraphMap()
{
    identity_orientation_ = false;
    currentNode_ = NULL;
    prevNode_ = NULL;
    // Vector of all nodes in graph
    nodes_.clear();
    factors_.clear();
    map_nodes_.clear();
    mapparam_ = NULL;
    setMapFileID();
}

MapNodePtr GraphMap::GetCurrentNode()
{
    return currentNode_;
}

MapTypePtr GraphMap::GetCurrentMapNode()
{
    if (currentNode_->isOnDisk())
    {
        bool success = currentNode_->loadFromDisk();
        if (!success)
        {
            std::cout << "no success" << std::endl;
            RemoveNode(currentNode_);
            currentNode_ = prevNode_;
            return NULL;
        }
    }
    return currentNode_->GetMap();
}

MapNodePtr GraphMap::GetPreviousNode()
{
    return prevNode_;
}

void GraphMap::RemoveNode(NodePtr node)
{
    for (int i = 0; i < nodes_.size(); i++)
    {
        if (nodes_[i] == node)
        {
            nodes_.erase(nodes_.begin() + i);
        }
    }
    for (int i = 0; i < map_nodes_.size(); i++)
    {
        if (map_nodes_[i] == node)
        {
            map_nodes_.erase(map_nodes_.begin() + i);
        }
    }
    std::cout << "removed node" << std::endl;
}

// Add node with link uncertainty
void GraphMap::AddMapNode(const Affine3d& diff, const Matrix6d& cov)
{
    Affine3d newNodePose = Affine3d::Identity();
    Affine3d prevNodePose = GetCurrentNodePose();
    Affine3d Tdiff = Eigen::Affine3d::Identity();

    if (identity_orientation_)
    {
        Tdiff.translation() = diff.translation();
        newNodePose = GetCurrentNodePose() * Tdiff;
    }
    else
    {
        Tdiff = diff;
        newNodePose = prevNodePose * Tdiff;
    }

    MapNodePtr newNode = GraphFactory::CreateMapNode(newNodePose, mapparam_, map_dir_);
    cout << "new map node: " << newNode->GetId() << ", prev: " << currentNode_->GetId() << endl;
    FactorPtr sd = GraphFactory::CreateMapNodeFactor(currentNode_, newNode, Tdiff, cov);
    // Add connection between current and new node with link diff and covariance
    factors_.push_back(sd);
    nodes_.push_back(newNode);
    map_nodes_.push_back(newNode);
    prevNode_ = currentNode_;
    currentNode_ = newNode;

    // note unloading
    if (map_unloading_)
    {
        if (!prevNode_->isOnDisk())
        {
            prevNode_->unloadToDisk(getMapFileID());
        }
    }
}

void GraphMap::AddMapNodeExplicitPar(MapParamPtr par_ptr,
                                     const Eigen::Affine3d& Tnode,
                                     const Matrix6d& cov)
{
    Eigen::Affine3d Tinit = Eigen::Affine3d::Identity();
    if (identity_orientation_)
    {
        Tinit.translation() = Tnode.translation();
    }
    else
    {
        Tinit = Tnode;
    }

    MapNodePtr newNode = GraphFactory::CreateMapNode(Tinit, par_ptr, map_dir_);
    if (currentNode_ != NULL)
    {
        Eigen::Affine3d Tdiff = GetCurrentNodePose().inverse() * Tinit;
        FactorPtr f = GraphFactory::CreateMapNodeFactor(currentNode_, newNode, Tdiff, unit_covar);
        factors_.push_back(f);
    }
    nodes_.push_back(newNode);
    map_nodes_.push_back(newNode);
    prevNode_ = currentNode_;
    currentNode_ = newNode;
}

void GraphMap::AddFixedConstraint(MapNodePtr mapnode,
                                  const Eigen::Affine3d& Tdiff,
                                  const Matrix6d& cov)
{
    NodePtr fixnode = NodePtr(new Node());
    Eigen::Affine3d Tnode = mapnode->GetMapPose() * Tdiff;
    fixnode->SetPose(Tnode);
    FactorPtr constraint = FactorPtr(new factor(mapnode, fixnode, Tdiff, cov));
    fixed_nodes_.push_back(fixnode);
    factors_.push_back(constraint);
}

FactorPtr GraphMap::FindLink(MapNodePtr prev, MapNodePtr next)
{
    for (int i = 0; i < factors_.size(); i++)
    {
        if (factors_[i]->Connects(prev) && factors_[i]->Connects(next))
        {
            return factors_[i];
        }
    }
    return NULL;
}

void GraphMap::UpdateLink(MapNodePtr prev,
                          MapNodePtr next,
                          const Affine3d& Tdiff,
                          const Matrix6d& cov)
{

    next->SetPose(prev->GetPose() * Tdiff);
    for (int i = 0; i < factors_.size(); i++)
    {
        if (factors_[i]->Connects(prev) && factors_[i]->Connects(next))
        {
            factors_[i]->UpdateFactor(Tdiff);
            cout << "Update Link: " << i << endl;
        }
    }
}

void GraphMap::UpdateAllNodePoses()
{
    for (mapNodeItr itr = this->begin(); itr != this->end(); itr++)
    {
        Eigen::Affine3d node_pose;
        if (itr == this->begin())
        {
            node_pose = (*itr)->GetPose(); // get absolute pose of the first node
        }
        else
        {
            bool status;
            FactorPtr f = this->FindLink(*(itr - 1), *itr);
            node_pose =
                node_pose * f->GetTransform(*(itr - 1), *(itr),
                                            status); // get relative transformation between nodes

            if (!status)
            {
                cerr << "Transformation not found netween \"" << GetNodeLinkName(*(itr - 1))
                     << "\" and \"" << GetNodeLinkName(*itr) << "\"" << endl;
            }
        }
        (*itr)->SetPose(node_pose);
    }
}

string GraphMap::ToString()
{
    std::stringstream ss;
    ss << endl << "GraphMap:" << endl;
    ss << "Graph size=" << nodes_.size() << endl;
    ss << "identity_orientation=" << std::boolalpha << identity_orientation_ << endl;
    for (int i = 0; i < nodes_.size(); i++)
    {
        if (i == 0)
        {
            ss << "Node positions:" << endl;
        }

        NodePtr ptr = nodes_[i];
        Eigen::Vector3d position = ptr->GetPose().translation();
        if (MapNodePtr ptr_mapnode = boost::dynamic_pointer_cast<MapNode>(ptr))
        {
            ss << "node " << ptr->GetId() << " (x,y,z):(" << position(0) << "," << position(1)
               << "," << position(2) << "), poses(est,gt)=(" << ptr_mapnode->GetPoseEst().size()
               << "," << ptr_mapnode->GetPoseExt().size() << ")" << endl;
        }
        else
        {
            ss << "node " << ptr->GetId() << " (x,y,z):(" << position(0) << "," << position(1)
               << "," << position(2) << endl;
        }
    }
    if (currentNode_ != NULL)
    {
        ss << "Detailed node info:" << endl << currentNode_->ToString();
    }

    return ss.str();
}

const Eigen::Affine3d& GraphMap::GetNodePose(int nodeNr) const
{
    if (nodeNr < Size())
    {
        return nodes_[nodeNr]->GetPose();
    }
}

Affine3d GraphMap::GetCurrentNodePose(bool use_obs_centroid)
{
    return currentNode_->GetMapPose(use_obs_centroid);
}

Affine3d GraphMap::GetPreviousNodePose(bool use_obs_centroid)
{
    return prevNode_->GetMapPose(use_obs_centroid);
}

MapNodePtr GraphMap::GetNode(unsigned int node_id)
{
    if (node_id < Size())
    {
        return map_nodes_[node_id];
    }
    else
    {
        return NULL;
    }
}

int GraphMap::GetNodeId(const MapNodePtr& node) const
{
    for (int i = 0; i < nodes_.size(); i++)
    {
        if (nodes_[i] == node)
        {
            return i;
        }
    }
}

void GraphMap::UpdatePoseFrame()
{
    for (int i = 0; i < map_nodes_.size(); i++)
    {
        map_nodes_[i]->UpdatePoseFrame();
    }
}

void GraphMap::GetCloud(pcl::PointCloud<pcl::PointXYZL>& cloud, bool downsampled)
{
    cloud.clear();
    for (int i = 0; i < map_nodes_.size(); i++)
    {
        if (map_nodes_[i]->isOnDisk())
        {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZL> curr_node_cloud;
        map_nodes_[i]->GetMap()->GetCloud(curr_node_cloud, downsampled);
        Eigen::Affine3d cloud_offset = map_nodes_[i]->GetPose();
        transformPointCloudInPlace(cloud_offset, curr_node_cloud);
        cloud += curr_node_cloud;
    }
}

void SaveGraphMapPCD(const std::string& path, const std::string& filename, GraphMapPtr graph_map)
{
    pcl::PointCloud<pcl::PointXYZL> cloud;
    cout << "Get cloud" << endl;
    graph_map->GetCloud(cloud, false);
    cout << "Saving cloud of size" << cloud.size() << endl;
    pcl::io::savePCDFileASCII(path + filename, cloud);
}

void GraphMap::AddFactor(MapNodePtr prev,
                         MapNodePtr next,
                         const Eigen::Affine3d& Tdiff,
                         const Matrix6d& cov)
{
    factors_.push_back(GraphFactory::CreateMapNodeFactor(prev, next, Tdiff, cov));
}

void GraphMap::setMapFileID()
{
    int len = 5;
    static bool set = false;
    if (not set)
    {
        srand(time(NULL));
        static const char alphanum[] = "0123456789"
                                       "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        std::string tmp_s;
        tmp_s.reserve(len);

        for (int i = 0; i < len; ++i)
        {
            tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
        }

        map_file_id_ = tmp_s;
        set = true;
    }
}

void GraphMap::setMapDirToAll(const std::string& path)
{
    setMapDir(path);
    std::vector<MapNodePtr> mapNodes = GetMapNodes();
    for (MapNodePtr node : mapNodes)
    {
        node->setPath(path);
    }
}

bool replace(std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

void GraphMap::RemoveNonStaticVoxels()
{
    std::cout << "removing all non-static voxels" << std::endl;

    const std::vector<int> nonstatic_labels_ = {0,  1,  10,  11,  13,  15,  16,  18,  20,  30,
                                                31, 32, 252, 253, 254, 255, 256, 257, 258, 259};
    int all = 0;
    int changed = 0;
    std::string id = getMapFileID();
    int idx = 0;
    for (MapNodePtr node : GetMapNodes())
    {
        if (node->isOnDisk())
        {
            bool success = false;
            success = node->loadFromDisk();
            if (!success)
            {
                std::cout << "node " << idx << " couldn't be loaded!" << std::endl;
                idx++;
                continue;
            }
        }
        idx++;

        NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(node->GetMap());
        NDTMap* ndt = current_map->GetNDTMap();
        std::pair<int, int> ret = ndt->removeVoxelsWithListedLabels(nonstatic_labels_);
        changed += ret.first;
        all += ret.second;

        node->unloadToDisk(id);
    }

    double p = (double)changed / (double)all * 100.0;
    std::cout << "removed " << changed << " cells out of " << all << std::endl;
    std::cout << "that is " << p << " % of cells" << std::endl;
}

void GraphMap::ClusterMap(bool viewOnly, bool clusterSemiStatic, bool clusterStatic)
{
    std::cout << "creating clusters" << std::endl;

    std::string id = getMapFileID();
    int idx = 0;
    for (MapNodePtr node : GetMapNodes())
    {
        if (node->isOnDisk())
        {
            bool success = false;
            success = node->loadFromDisk();
            if (!success)
            {
                std::cout << "node " << idx << " couldn't be loaded!" << std::endl;
                idx++;
                continue;
            }
        }
        idx++;

        NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(node->GetMap());
        NDTMap* ndt = current_map->GetNDTMap();
        if (!viewOnly)
        {
            ndt->cluster(true, clusterSemiStatic, clusterStatic);
        }
        std::map<int, std::vector<NDTCell*>> clusters = ndt->getClusters();

        std::cout << "map id: " << idx << " has " << clusters.size() << " clusters" << std::endl;
        std::map<int, std::vector<NDTCell*>>::iterator it;

        node->unloadToDisk(id);
    }

    std::cout << "clustered all maps" << std::endl;
}

DynamicStatistics GraphMap::GetDynamicStatistics()
{
    DynamicStatistics stats;
    std::cout << "unload all maps" << std::endl;
    for (MapNodePtr map : map_nodes_)
    {
        bool onDisk = map->isOnDisk();
        if (not onDisk)
        {
            map->unloadToDisk(getMapFileID());
        }
    }

    std::string id = getMapFileID();
    int idx = 0;
    for (MapNodePtr node : GetMapNodes())
    {
        std::cout << "count node " << idx << std::endl;
        if (node->isOnDisk())
        {
            bool success = false;
            success = node->loadFromDisk();
            if (!success)
            {
                std::cout << "node " << idx << " couldn't be loaded!" << std::endl;
                idx++;
                continue;
            }
        }
        idx++;
        DynamicStatistic stat = GetSigleMapDynamicStatistics(node);
        stats.add(stat);

        node->unloadToDisk(id);
    }
    stats.count();

    std::cout << stats.getDynamic() << " dynamic labels: " << stats.getPDynamic() << " %" << std::endl;
    std::cout << stats.getSemistatic() << " semistatic labels: " << stats.getPSemistatic() << " %" << std::endl;
    std::cout << stats.getStatic() << " static labels: " << stats.getPStatic() << " %" << std::endl;
    std::cout << stats.getUnknown() << " unknown labels: " << stats.getPUnknown() << " %" << std::endl;

    return stats;
}

DynamicStatistic GraphMap::GetSigleMapDynamicStatistics(MapNodePtr node)
{
    DynamicStatistic stat;
    if (!node)
    {
        node = GetCurrentNode();
    }
    const std::vector<int> static_labels_ = {40, 44, 48, 49, 50, 51, 52,
                                             60, 70, 71, 72, 80, 81, 99};
    const std::vector<int> semistatic_labels_ = {
        10, 11, 13, 15, 16, 18, 20, 30, 31, 32,
    };
    const std::vector<int> dynamic_labels_ = {252, 253, 254, 255, 256, 257, 258, 259};

    NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(node->GetMap());
    NDTMap* ndt = current_map->GetNDTMap();
    stat.dynamic_ = ndt->countListedLabels(dynamic_labels_);
    stat.semistatic_ = ndt->countListedLabels(semistatic_labels_);
    stat.static_ = ndt->countListedLabels(static_labels_);
    std::vector<NDTCell*> allCells = ndt->getAllCells();
    int all = allCells.size();
    stat.unknown_ = all - stat.dynamic_ - stat.semistatic_ - stat.static_;

    std::cout << "dynamic labels: " << stat.dynamic_ << std::endl;
    std::cout << "semistatic labels: " << stat.semistatic_ << std::endl;
    std::cout << "static labels: " << stat.static_ << std::endl;
    std::cout << "unknown labels: " << stat.unknown_ << std::endl;
    std::cout << "total: " << all << std::endl;

    return stat;
}

GraphMapParam::GraphMapParam()
{
}

void GraphMapParam::GetParametersFromRos()
{
}

} // namespace graph_map
} // namespace perception_oru
