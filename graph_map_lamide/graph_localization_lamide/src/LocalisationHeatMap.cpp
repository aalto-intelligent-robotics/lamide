#include "graph_localization_lamide/LocalisationHeatMap.h"
namespace HeatMap
{

LocalisationHeatMap::LocalisationHeatMap(double resolution,
                                         const double map_size_x,
                                         const double map_size_y,
                                         const double map_size_z)
{
    sizeXmeters = map_size_x;
    sizeYmeters = map_size_y;
    sizeZmeters = map_size_z;

    half_sizeXmeters = map_size_x / 2.0;
    half_sizeYmeters = map_size_y / 2.0;
    half_sizeZmeters = map_size_z / 2.0;

    resolution_ = resolution;
    sizeX = abs(ceil(sizeXmeters / resolution_));
    sizeY = abs(ceil(sizeYmeters / resolution_));
    sizeZ = abs(ceil(sizeZmeters / resolution_));
    std::cout << "Create new grid with number of cells: (sizeX x sizeY x sizeZ)=(" << sizeX << " x "
              << sizeY << " x " << sizeZ << " )" << std::endl;

    half_voxel = resolution_ / 2.0;
    dataArray = new Voxel***[sizeX];
    for (int i = 0; i < sizeX; i++)
    {
        dataArray[i] = new Voxel**[sizeY];
        for (int j = 0; j < sizeY; j++)
        {
            dataArray[i][j] = new Voxel*[sizeZ];
            memset(dataArray[i][j], 0, sizeZ * sizeof(Voxel*)); // set all cells to NULL
        }
    }
}

float LocalisationHeatMap::GetScore(const Eigen::Vector3d& p)
{
    Eigen::Vector3i idx;
    if (!getIndexForPoint(p, idx))
        return -1.0;

    if (dataArray[idx(0)][idx(1)][idx(2)] != NULL)
        return dataArray[idx(0)][idx(1)][idx(2)]->val;
    else
        return -1.0;
}
bool LocalisationHeatMap::getIndexForPoint(const Eigen::Vector3d& p, Eigen::Vector3i& idx)
{
    // std::cout<<"p(0)+half_sizeXmeters)/resolution
    // ="<<(p(0)+half_sizeXmeters)/resolution_<<std::endl;
    idx(0) = std::floor((p(0) + half_sizeXmeters) / resolution_);
    idx(1) = std::floor((p(1) + half_sizeYmeters) / resolution_);
    idx(2) = std::floor((p(2) + half_sizeZmeters) / resolution_);
    if (idx(0) >= sizeX || idx(0) < 0 || idx(1) >= sizeY || idx(1) < 0 || idx(2) >= sizeZ ||
        idx(2) < 0)
        return false;
    else
        return true;
    // cout<<"Index for: "<<p.transpose()<<", is: "<<idx.transpose()<<endl;
}

Eigen::Vector3d LocalisationHeatMap::GetVoxelCenter(const Eigen::Vector3i& idx)
{
    if (idx(0) >= sizeX || idx(0) < 0 || idx(1) >= sizeY || idx(1) < 0 || idx(2) >= sizeZ ||
        idx(2) < 0)
        return Eigen::Vector3d::Identity();
    // cout<<"center of:"<<idx.transpose()<<endl;
    Eigen::Vector3d center(idx(0) * resolution_ + half_voxel - half_sizeXmeters,
                           idx(1) * resolution_ + half_voxel - half_sizeYmeters,
                           idx(2) * resolution_ + half_voxel - half_sizeZmeters);
    // cout<<"is:"<<center.transpose()<<endl;
    return center;
}

Eigen::Vector3d LocalisationHeatMap::GetVoxelCenter(const IndexedVoxel& voxel)
{
    return GetVoxelCenter(voxel.second);
}
bool LocalisationHeatMap::getNeighborsByRadius(const double& radius,
                                               const Eigen::Vector3d& p,
                                               VoxelVector& nearby_voxels,
                                               bool OnlyInitialized)
{
    Eigen::Vector3i idx_min, idx_max;
    Eigen::Vector3d p_min(p(0) - radius, p(1) - radius, p(2) - radius);
    Eigen::Vector3d p_max(p(0) + radius, p(1) + radius, p(2) + radius);
    // cout<<"pmin: "<<p_min.transpose()<<", pmax: "<<p_max.transpose()<<endl;
    getIndexForPoint(p_min, idx_min);
    getIndexForPoint(p_max, idx_max);

    unsigned int count_voxels = 0;
    for (int x = idx_min(0); x <= idx_max(0); x++)
    {
        if (x < 0 || x >= sizeX)
            continue;
        for (int y = idx_min(1); y <= idx_max(1); y++)
        {
            if (y < 0 || y >= sizeY)
                continue;
            for (int z = idx_min(2); z <= idx_max(2); z++)
            {
                if (z < 0 || z >= sizeZ)
                    continue;
                if (dataArray[x][y][z] == NULL && OnlyInitialized)
                    continue;
                IndexedVoxel neighbor_voxel =
                    std::make_pair(dataArray[x][y][z], Eigen::Vector3i(x, y, z));
                nearby_voxels.push_back(neighbor_voxel);
                count_voxels++;
            }
        }
    }
    // cout<<"found voxels: "<<count_voxels<<endl;
    return true;
}
void LocalisationHeatMap::PrintGrid()
{

    for (int x = 0; x < sizeX; x++)
    {
        for (int y = 0; y < sizeY; y++)
        {
            for (int z = 0; z < sizeZ; z++)
            {
                if (dataArray[x][y][z] == NULL)
                {
                    // cout<<"NULL"<<endl;
                    continue;
                }
                cout << dataArray[x][y][z]->val << endl;
            }
        }
    }
}
void LocalisationHeatMap::InitializeNeighborsByRadius(const double& radius,
                                                      const Eigen::Vector3d& p,
                                                      VoxelVector& nearby_voxels)
{

    nearby_voxels.clear();
    getNeighborsByRadius(radius, p, nearby_voxels, false);
    for (auto&& i : nearby_voxels)
    { // access by value, the type of i is int
        if (i.first == NULL)
        {
            // cout<<"initialize: "<<i.second(0)<<", "<<i.second(1)<<", "<<i.second(2)<<endl;
            dataArray[i.second(0)][i.second(1)][i.second(2)] = Voxel::CreateVoxel();
            i.first = dataArray[i.second(0)][i.second(1)][i.second(2)];
        }
    }
    //
    // cout<<"Created "<<count<<" new voxels"<<endl;
}

void LocalisationHeatMap::InitializeNeighborsByRadius(const double& radius,
                                                      const Eigen::Vector3d& p)
{
    VoxelVector nearby_voxels;
    InitializeNeighborsByRadius(radius, p, nearby_voxels);
}

void LocalisationHeatMap::VoxelFilter(VoxelVector& vek, bool remove_initialized_voxels)
{
    VoxelVector vek_new;
    if (remove_initialized_voxels)
    {
        for (auto& i : vek)
        {
            if (i.first == NULL)
                vek_new.push_back(i);
        }
        vek.clear();
        vek = vek_new;
    }
}
double LocalisationHeatMap::DistanceP2VCenter(const Eigen::Vector3d& p, const IndexedVoxel& voxel)
{
    Eigen::Vector3d center = GetVoxelCenter(voxel);
    double d = (center - p).norm();
    // cout<<"distance between"<<p.transpose()<<" and "<<center<<" is "<<d<<endl;
    return d;
}

void LocalisationHeatMap::UpdateHeatmapNormal(const Eigen::Vector3d& Tupdsrc, double sigma_update)
{
    VoxelVector nearby_voxels;
    InitializeNeighborsByRadius(limit_ * sigma_update, Tupdsrc, nearby_voxels);
    // cout<<"radius "<<limit_*sigma_update<<" around :"<<Tupdsrc.transpose()<<", there are now:
    // "<<nearby_voxels.size()<<" initialized voxels"<<endl;
    const double two_sigma_squared = 2 * sigma_update * sigma_update;
    const double normalizer = 1 / (sqrt(two_sigma_squared * M_PI));
    for (auto& v : nearby_voxels)
    {
        double d = DistanceP2VCenter(Tupdsrc, v);
        float p = normalizer * exp(-(d * d) / two_sigma_squared);
        if (v.first == NULL)
        {
            cerr << "Error initializing voxels" << endl;
            exit(0);
        }
        v.first->val += p;
        n_updates_++;
    }
}

HeatMapInterface::HeatMapInterface(pogm::GraphMapNavigatorPtr graph_map, const double resolution)
{
    graph_map_ = graph_map;
    res_ = resolution;
    UpdateHeatMaps();
    initialized_ = true;
}
void HeatMapInterface::UpdateHeatMaps()
{
    std::mutex m;
    heatmaps_.resize(graph_map_->Size());
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < graph_map_->Size(); i++)
        {
            m.lock();
            pogm::MapNodePtr nodePtr = graph_map_->GetNode(i);
            double map_sizex, map_sizey, map_sizez;
            nodePtr->GetMap()->GetSize(map_sizex, map_sizey, map_sizez);
            LocalisationHeatMap* lhm =
                new LocalisationHeatMap(res_, map_sizex, map_sizey, map_sizez);
            ndt_generic::Affine3dSTLVek obs_vek = nodePtr->GetObservationVector(true);
            m.unlock();
            // cout<<"update map: "<<nodePtr->GetId()<<", with vector of size:
            // "<<obs_vek.size()<<endl;
            for (auto& obs : obs_vek)
            {
                // cout<<"obs: "<<obs.translation().transpose()<<endl;
                lhm->UpdateHeatmapNormal(obs.translation(), sigma1_);
                lhm->UpdateHeatmapNormal(obs.translation(), sigma2_);
            }
            heatmaps_[i] = std::make_pair(lhm, nodePtr->GetId());
        }
    }
}

float HeatMapInterface::GetMapForPoint(const Eigen::Vector3d& p,
                                       pogm::MapNodePtr& map_ptr,
                                       pogm::GraphMapNavigatorPtr graph_map)
{
    map_ptr = NULL;
    float max_score;
    if (heatmaps_.size() == 0)
        return -1;
    if (heatmaps_.size() >= 1)
    {
        map_ptr = graph_map->GetNode(heatmaps_.front().second);
        if (map_ptr == NULL)
        {
            cerr << "Null map pointer" << endl;
            exit(0);
        }
        Eigen::Vector3d p_local = map_ptr->GetPose().inverse() * p;
        max_score = heatmaps_.front().first->GetScore(p_local);
    }

    // cout<<"score in first: "<<max_score<<endl;
    for (auto& pair : heatmaps_)
    {
        float tmp_score = -1;
        pogm::MapNodePtr tmp_node = graph_map->GetNode(pair.second);
        tmp_score = pair.first->GetScore(tmp_node->GetPose().inverse() * p);
        // cout<<"map: "<<pair.second->GetId()<<" = "<<tmp_score<<endl;
        if (tmp_score > max_score)
        {
            max_score = tmp_score;
            map_ptr = tmp_node;
        }
    }
    return max_score;
}

bool HeatMapInterface::SetAndVerifyInterface(pogm::GraphMapNavigatorPtr& graph)
{

    graph_map_ = graph;
    if (graph_map_->Size() == heatmaps_.size())
    {
        initialized_ = true;
        return true;
    }
    else
        return false;
}

bool HeatMapInterface::LoadHeatMap(const std::string& file_name,
                                   boost::shared_ptr<HeatMapInterface>& ptr)
{

    if (file_name == "")
    {
        cerr << "No file path provided" << endl;
        return false;
    }
    std::ifstream f(file_name.c_str());
    if (!f.good())
    {
        cerr << "The file  \"" << file_name << "\" does not exist." << endl;
        return false;
    }

    try
    {
        cout << "Loading file at path: \n" << file_name << endl;
        std::ifstream ifs(file_name);
        boost::archive::binary_iarchive ia(ifs);
        ia& ptr;
        cout << "The following file was loaded:\n" << ptr->ToString() << endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading file at path:\n" << file_name << std::endl;
        std::cerr << e.what() << endl;
        return false;
    }
    if (ptr == NULL)
    {
        std::cerr << "Error loading file" << std::endl;
        return false;
    }
    return true;
}
void HeatMapInterface::SaveHeatMap(const std::string& file_name,
                                   boost::shared_ptr<HeatMapInterface>& ptr)
{

    cout << "-----------------------------Saving---------------------------------\n"
         << ptr->ToString() << endl;
    cout << "----------------------------------------------------------------------\nTo file path:"
         << file_name << endl;
    try
    {
        std::ofstream ofs(file_name);
        boost::archive::binary_oarchive ar(ofs);
        ar << ptr;
        ofs.close();
    }
    catch (std::exception e)
    {
        std::cerr << e.what() << endl;
    }
}

std::string HeatMapInterface::ToString()
{
    std::stringstream ss;

    ss << "MapNodes: " << std::to_string(heatmaps_.size()) << endl;
    for (int i = 0; i < heatmaps_.size(); i++)
    {
        if (heatmaps_[i].first == NULL)
            ss << "NULL, ";
        else
            ss << "Upd:" << heatmaps_[i].first->GetUpdates() << ", id:" << heatmaps_[i].second
               << ", ";
    }
    ss << "\n Nodes: " + std::to_string(heatmaps_.size()) + "\n";
    return ss.str();
}

} // namespace HeatMap
