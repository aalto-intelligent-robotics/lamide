#include "graph_map_lamide/map_node.h"

#include <boost/serialization/export.hpp>
#include <chrono>

BOOST_CLASS_EXPORT(perception_oru::graph_map::MapNode)
namespace perception_oru
{
namespace graph_map
{

std::vector<Eigen::Affine3d> getAffineOffsets(const std::vector<Eigen::Affine3d>& Ts, double offset)
{
    std::vector<Eigen::Affine3d> Ts_out;

    for (size_t i = 0; i < Ts.size(); i++)
    {
        Ts_out.push_back(Ts[i] * ndt_generic::xyzrpyToAffine3d(offset, 0., 0., 0., 0., 0.));
        Ts_out.push_back(Ts[i] * ndt_generic::xyzrpyToAffine3d(-offset, 0., 0., 0., 0., 0.));
        Ts_out.push_back(Ts[i] * ndt_generic::xyzrpyToAffine3d(0., offset, 0., 0., 0., 0.));
        Ts_out.push_back(Ts[i] * ndt_generic::xyzrpyToAffine3d(0., -offset, 0., 0., 0., 0.));
        Ts_out.push_back(Ts[i] * ndt_generic::xyzrpyToAffine3d(0., 0., offset, 0., 0., 0.));
        Ts_out.push_back(Ts[i] * ndt_generic::xyzrpyToAffine3d(0., 0., -offset, 0., 0., 0.));
    }
    return Ts_out;
}

Eigen::Affine3d alignPoses(const std::vector<Eigen::Affine3d>& targetTs_,
                           const std::vector<Eigen::Affine3d>& sourceTs_,
                           double offset)
{
    cout << "getting poses ALIGN poses target:" << targetTs_.size()
         << ", source:" << sourceTs_.size() << endl;
    if (targetTs_.size() != sourceTs_.size())
    {
        std::cerr << "target and source vectors are diffent(!)" << __FUNCTION__ << std::endl;
        exit(-1);
    }

    std::vector<Eigen::Affine3d> targetTs = getAffineOffsets(targetTs_, offset);
    std::vector<Eigen::Affine3d> sourceTs = getAffineOffsets(sourceTs_, offset);

    Eigen::Vector3d c0;
    Eigen::Vector3d c1;
    Eigen::Matrix3d H;
    {
        c0.setZero();
        c1.setZero();

        // get centroids
        for (size_t i = 0; i < targetTs.size(); i++)
        {
            c0 += targetTs[i].translation(); // Fix
            c1 += sourceTs[i].translation(); // Moving
        }
        c0 *= (1. / (1. * targetTs.size()));
        c1 *= (1. / (1. * targetTs.size()));

        Eigen::MatrixXd target(targetTs.size(), 3);
        Eigen::MatrixXd source(sourceTs.size(), 3);
        for (size_t i = 0; i < targetTs.size(); i++)
        {
            target.row(i) = targetTs[i].translation() - c0;
            source.row(i) = sourceTs[i].translation() - c1;
        }

        H = target.transpose() * source;
    }

    // do the SVD thang
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * svd.matrixU().transpose();
    double det = R.determinant();
    // ntot++;
    if (det < 0.0)
    {
        // nneg++;
        V.col(2) = V.col(2) * -1.0;
        R = V * svd.matrixU().transpose();
    }
    Eigen::Vector3d tr = c0 - R.transpose() * c1; // translation

    std::cout << "translation : " << c0 - c1 << std::endl;
    std::cout << "tr : " << tr << std::endl;
    std::cout << "c0-R.transpose()*c1 : " << c0 - R.transpose() * c1 << std::endl;
    // transformation matrix, 3x4
    Eigen::Affine3d T;
    T.linear() = R.inverse();
    T.translation() = tr;

    return T;
}

Node::Node()
{
    static unsigned int num_nodes = 0;
    id_ = num_nodes;
    num_nodes++;
    pose_ = Eigen::Affine3d::Identity();
}

Affine3d& Node::GetPose()
{
    return pose_;
}

bool Node::operator==(const Node& node_compare)
{
    if (id_ == node_compare.id_ && pose_.isApprox(node_compare.pose_))
        return true;
    else
        return false;
}

bool Node::WithinRadius(const Affine3d& pose, const double& radius)
{

    double distance = Eigen::Vector3d(pose.translation() - pose_.translation()).norm();
    if (distance < radius)
        return true;
    else
        return false;
}

bool Node::DetectNewNode(unsigned int& node_id, const NodePtr& node)
{ // compare node with node_id, if different, node.GetId is assigned to node_id
    if (node->GetId() == node_id)
        return false;
    else
    {
        node_id = node->GetId();
        return true;
    }
}

MapNode::MapNode(const Eigen::Affine3d& pose, const MapParamPtr& mapparam, const std::string& path)
{
    pose_ = pose;
    map_ = GraphFactory::CreateMapType(mapparam);
    path_ = path;
}
MapNode::MapNode()
{
    map_ = NULL;
}
Eigen::Affine3d MapNode::GetMapPose(bool obs_centroid)
{
    if (!obs_centroid)
        return GetPose();
    else
        return GetPose() * map_->GetObservationCentroid();
}
bool MapNode::WithinRadius(const Affine3d& pose, const double& radius, bool obs_centroid)
{
    double distance =
        Eigen::Vector3d(pose.translation() - GetMapPose(obs_centroid).translation()).norm();
    if (distance < radius)
        return true;
    else
        return false;
}
string MapNode::ToString()
{
    std::stringstream ss;
    ss << "MapNode:\ninitialized: " << initialized_ << "\nid:" << id_
       << "\nPosition(x,y,z,r,p,y):" << ndt_generic::affine3dToStringRPY(pose_) << endl;
    ;
    if (map_ != NULL)
        ss << map_->ToString();
    return ss.str();
}

void MapNode::updateMap(const Eigen::Affine3d& Tnow,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple,
                        bool cluster)
{
    map_->updateMap(Tnow, cloud, simple, cluster);
    initialized_ = true;
}

void MapNode::updateMap(const Eigen::Affine3d& Tnow,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple,
                        bool cluster)
{
    map_->updateMap(Tnow, clouds, simple, cluster);
    initialized_ = true;
}

void MapNode::AddPoseFramePair(const Eigen::Affine3d& Tnow, const Eigen::Affine3d& Tref)
{
    poseFrames_.push_back(Tnow);
    poseFramesExternal_.push_back(Tref);
}
std::vector<Eigen::Affine3d> MapNode::GetPoseEst()
{
    return poseFrames_;
}
std::vector<Eigen::Affine3d> MapNode::GetPoseExt()
{
    return poseFramesExternal_;
}

void MapNode::UpdatePoseFrame()
{
    // Align the pose pairs and set a new node pose.
    if (poseFrames_.empty())
        return;

    if (poseFrames_.size() != poseFramesExternal_.size())
        return;

    this->SetPose(alignPoses(poseFramesExternal_, poseFrames_, 0.5));
}

ndt_generic::Affine3dSTLVek MapNode::GetObservationVector(bool local_frame)
{
    ndt_generic::Affine3dSTLVek obs_vec = map_->GetObservationVector();
    if (!local_frame)
    {
        for (int i = 0; i < obs_vec.size(); i++)
            obs_vec[i] = GetPose() * obs_vec[i];
    }
    return obs_vec;
}
std::string GetNodeLinkName(const MapNodePtr& node)
{
    return std::string(NODE_LINK_ID_PREFIX + std::to_string(node->GetId()));
}

std::string MapNode::getUnloadFile() const
{
    return unloadFile_;
}

void MapNode::setUnloadFile(const std::string& newFile)
{
    unloadFile_ = newFile;
}

void MapNode::fixPathStrings()
{
    size_t pos = unloadFile_.find(path_);
    if (pos != std::string::npos)
    {
        std::cout << "unload file has absolute path, fixing..." << std::endl;
        unloadFile_.erase(pos, path_.length());
    }
    pos = unloadFile_.find_last_of("\\/");
    if (pos != std::string::npos)
    {
        std::string problemPath = unloadFile_.substr(0, pos);
        std::cout << "unload file has absolute path, fixing..." << std::endl;
        unloadFile_.erase(0, problemPath.length() + 1);
    }

    // if the path_ is missing slash, add it
    if (path_.back() != '/')
    {
        path_ = path_ + "/";
    }
}

bool MapNode::unloadToDisk(const std::string& mapid)
{
    unsigned int id = GetId();
    if (map_ == NULL)
    {
        std::cout << "Failed to unload submap " << id << " to disk!" << std::endl;
        return false;
    }
    if (path_.empty() && mapid.empty() && unloadFile_.empty())
    {
        std::cout << "Cannot load map, no path given!!" << std::endl;
        return false;
    }

    fixPathStrings();

    // set vars
    std::string fullpath = path_ + unloadFile_;
    if (unloadFile_.empty())
    {
        auto stamp = std::chrono::system_clock::now().time_since_epoch().count();

        std::stringstream localpath;
        localpath << mapid << "_submap_" << id << "_" << stamp;
        unloadFile_ = localpath.str();
        fullpath = path_ + unloadFile_;
    }
    onDisk_ = true;

    // unload
    std::ofstream ofs(fullpath);
    boost::archive::binary_oarchive ar(ofs);
    ar << map_;
    ofs.close();

    map_.reset();
    map_ = NULL;

    std::cout << "unloaded map " << id << " to disk: " << path_ << unloadFile_ << std::endl;
    return true;
}

bool MapNode::loadFromDisk()
{
    if (map_ != NULL)
    {
        std::cout << "map is not null!" << std::endl;
    }

    fixPathStrings();

    try
    {
        std::ifstream ifs(path_ + unloadFile_);
        if(ifs.is_open())
        {
            boost::archive::binary_iarchive ia(ifs);
            ia& map_;
        }
        else
        {
            std::cout << "problem opening the file stream!" << std::endl;
        }
        ifs.close();
    }
    catch(const std::exception& e)
    {
        std::cout << "Error loading the map: " << std::endl;
        std::cout << e.what() << std::endl;
        std::cout << "path: " << path_ << std::endl;
        std::cout << "unload file: " << unloadFile_ << std::endl;
    }

    if (map_ != NULL)
    {
        unsigned int id = GetId();
        std::cout << "map " << id << " loaded from disk: " << path_ << unloadFile_ << std::endl;
        onDisk_ = false;
        return true;
    }
    else
    {
        std::cout << "problem loading map from disk!" << std::endl;
        return false;
    }
}

} // namespace graph_map
} // namespace perception_oru
