#include "graph_map_lamide/graph_map_navigator.h"

#include <boost/serialization/export.hpp>
#include <iomanip>
#include <graph_map_lamide/graphfactory.h>

BOOST_CLASS_EXPORT(perception_oru::graph_map::GraphMapNavigator)
namespace perception_oru
{
namespace graph_map
{
GraphMapNavigator::GraphMapNavigator(const Eigen::Affine3d& nodepose,
                                     const MapParamPtr& mapparam,
                                     const GraphMapParamPtr graphparam)
    : GraphMap(nodepose, mapparam, graphparam)
{
    if (GraphMapNavigatorParamPtr par =
            boost::dynamic_pointer_cast<GraphMapNavigatorParam>(graphparam))
    {
        use_submap_ = par->use_submap;
        check_for_broken_submaps_ = par->check_for_broken_submaps;
        interchange_radius_ = par->interchange_radius;
        compound_radius_ = par->compound_radius;
        use_keyframe_ = par->use_keyframe;
        min_keyframe_dist_ = par->min_keyframe_dist;
        min_keyframe_rot_deg_ = par->min_keyframe_rot_deg;
        map_switch_method_ = par->map_switch_method;
        n_search_ = par->n_search;
        Tsensor_ = par->Tsensor;
        cout << "Created GraphMapNavigator" << endl;
    }
    else
    {
        cerr << "Cannot create GraphMapNavigator" << endl;
        exit(0);
    }
}

GraphMapNavigator::GraphMapNavigator(const GraphMapParamPtr graphparam)
    : GraphMap(graphparam)
{
    if (GraphMapNavigatorParamPtr par =
            boost::dynamic_pointer_cast<GraphMapNavigatorParam>(graphparam))
    {
        use_submap_ = par->use_submap;
        interchange_radius_ = par->interchange_radius;
        compound_radius_ = par->compound_radius;
        use_keyframe_ = par->use_keyframe;
        min_keyframe_dist_ = par->min_keyframe_dist;
        min_keyframe_rot_deg_ = par->min_keyframe_rot_deg;
        map_switch_method_ = par->map_switch_method;
        n_search_ = par->n_search;
        Tsensor_ = par->Tsensor;
        cout << "Created GraphMapNavigator" << endl;
    }
    else
    {
        cerr << "Cannot create GraphMapNavigator" << endl;
        exit(0);
    }
}

void GraphMapNavigator::WorldToLocalMapFrame(Eigen::Affine3d& pose, MapNodePtr frame)
{
    if (frame == NULL)
        pose = GetCurrentNodePose().inverse() * pose;
    else
        pose = frame->GetPose().inverse() * pose;
}
void GraphMapNavigator::LocalToWorldMapFrame(Eigen::Affine3d& pose, MapNodePtr frame)
{
    if (frame == NULL)
        pose = GetCurrentNodePose() * pose;
    else
        pose = frame->GetPose() * pose;
}

bool GraphMapNavigator::SwitchToClosestMapNode(const Affine3d& Tnow,
                                               pcl::PointCloud<pcl::PointXYZL>& cloud,
                                               double max_distance,
                                               int count)
{
    MapNodePtr closest;
    if (map_switch_method_ == overlap)
    {
        SwitchToMapNode(GetMapByOverlap(Tnow, cloud, count));
    }
    else
    {
        SwitchToClosestMapNode(Tnow, max_distance);
    }
}

bool GraphMapNavigator::SwitchToClosestMapNode(const Affine3d& Tnow, double max_distance)
{
    double min_dist = DBL_MAX;
    MapNodePtr closest_map_node = NULL;

    if (map_switch_method_ == node_position)
    {
        closest_map_node = GetClosestMapNode(Tnow);
        if (closest_map_node->WithinRadius(Tnow, max_distance))
        {
            return SwitchToMapNode(closest_map_node);
        }
        else
        {
            return false;
        }
    }
    else
    {
        closest_map_node = GetMapNodeByObservationHistory(Tnow, n_search_);
        return SwitchToMapNode(closest_map_node);
    }
}
std::vector<MapNodePtr> GraphMapNavigator::GetClosestNodes(const Eigen::Affine3d& Tnow,
                                                           double max_distance,
                                                           bool factor_interchange)
{
    double distance = max_distance;
    if (factor_interchange)
        distance = interchange_radius_ * max_distance;

    std::vector<MapNodePtr> close_nodes;
    bool centroid = (map_switch_method_ == mean_observation) ? true : false;

    for (std::vector<MapNodePtr>::iterator itr_node = map_nodes_.begin();
         itr_node != map_nodes_.end(); ++itr_node)
    { // loop thorugh all mapnodes and list the one closer than max_distance
        if (ndt_generic::GetDistance((*itr_node)->GetMapPose(false), Tnow) < distance)
            close_nodes.push_back(*itr_node);
    }
    return close_nodes;
}
MapNodePtr GraphMapNavigator::GetMapByOverlap(const Eigen::Affine3d& Tnow,
                                              pcl::PointCloud<pcl::PointXYZL>& cloud,
                                              int count)
{
    // Define a sorting function
    struct SortFunction
    {
        SortFunction(const Eigen::Affine3d& Tnow)
        {
            this->T = Tnow;
        }
        bool operator()(MapNodePtr i, MapNodePtr j)
        {
            return (T.translation() - i->GetPose().translation()).norm() <
                   (T.translation() - j->GetPose().translation()).norm();
        }
        Eigen::Affine3d T;
    };

    // Set parameters for scan
    MapParamPtr par = currentNode_->GetMap()->GetPrototype()->clone();
    par->sizey_ = 100;
    par->sizex_ = 100;
    par->sizez_ = 10;
    par->store_points = false;
    par->enable_mapping_ = true;
    MapTypePtr scan = GraphFactory::CreateMapType(par);
    scan->updateMap(Eigen::Affine3d::Identity(), cloud);

    std::vector<MapNodePtr> close_nodes = GetClosestNodes(Tnow, 2 * interchange_radius_);
    if (close_nodes.size() == 0)
    {
        return GetClosestMapNode(Tnow);
    }

    std::sort(close_nodes.begin(), close_nodes.end(), SortFunction(Tnow));
    int id = currentNode_->GetId();
    int newId;
    MapNodePtr map_highest_l2 = NULL;
    double max_overlap = 0;
    for (int i = 0; i < close_nodes.size(); i++)
    {
        int oid = close_nodes[i]->GetId();

        Eigen::Affine3d Trobot_local =
            close_nodes[i]->GetPose().inverse() *
            (Tnow * Tsensor_.inverse()); // The transformation which takes  the cloud from Robot
                                         // in world into sensor in local map
        double distance = (Tnow.translation() - close_nodes[i]->GetPose().translation()).norm();
        double overlap = close_nodes[i]->GetMap()->Overlap(scan, Trobot_local);
        overlap = overlap * overlap;

        // penalize long jump in indices
        double idcoef = -250.0 * (double)(abs(oid - id));
        // slightly favor current map
        double current_bonus = oid == id ? 10000 : 0;
        // visitation factors
        double visit_penalty = oid != id ? -10000.0 * visited_nodes_[oid] : 0;
        double prev_penalty = oid == previous_map_id_ ? -100000.0 : 0.0;

        // HACK: initialization to zero
        double init_bonus = (count > 0 and count < 1000 and oid == 0) ? 50000 : 0;

        // total
        double total = overlap + idcoef + current_bonus + visit_penalty + prev_penalty + init_bonus;

        if (total > max_overlap)
        {
            max_overlap = total;
            map_highest_l2 = close_nodes[i];
            newId = oid;
        }
    }
    if (newId != id)
    {
        std::cout << "selected map " << newId << std::endl;
    }

    return map_highest_l2;
}
MapNodePtr GraphMapNavigator::GetMapByRegistration(Eigen::Affine3d& Tnow,
                                                   pcl::PointCloud<pcl::PointXYZL>& cloud,
                                                   RegTypePtr& regPtr)
{
    // Define a sorting function
    struct SortFunction
    {
        SortFunction(const Eigen::Affine3d& Tnow)
        {
            this->T = Tnow;
        }
        bool operator()(MapNodePtr i, MapNodePtr j)
        {
            return (T.translation() - i->GetPose().translation()).norm() <
                   (T.translation() - j->GetPose().translation()).norm();
        }
        Eigen::Affine3d T;
    };
    // Set parameters for scan

    std::vector<MapNodePtr> close_nodes = GetClosestNodes(Tnow, 3 * interchange_radius_);
    if (close_nodes.size() == 0)
        return GetClosestMapNode(Tnow);

    std::sort(close_nodes.begin(), close_nodes.end(), SortFunction(Tnow));

    MapNodePtr closest_map = close_nodes[0];
    double min_l2_score = DBL_MAX;
    Eigen::Affine3d Tbest_world = Tnow;
    for (int i = 0; i < 100 && i < close_nodes.size() && i < 2; i++)
    {
        Eigen::Affine3d Trobot_local =
            close_nodes[i]->GetPose().inverse() *
            (Tnow * Tsensor_.inverse()); // The transformation which takes  the cloud from Robot in
                                         // world into sensor in local map
        double distance = (Tnow.translation() - close_nodes[i]->GetPose().translation()).norm();
        ros::Time t = ros::Time::now();
        regPtr->RegisterScan(close_nodes[i]->GetMap(), Trobot_local, cloud);
        double score = regPtr->GetScore();
        if (score < min_l2_score)
        {
            Tbest_world = close_nodes[i]->GetPose() * Trobot_local * Tsensor_;
            min_l2_score = score;
            closest_map = close_nodes[i];
        }
    }
    cout << "Registration - overlap: selected node " << closest_map->GetId() << endl;
    Tnow = Tbest_world;
    return closest_map;
}
MapNodePtr GraphMapNavigator::GetMapNodeByObservationHistory(const Eigen::Affine3d& Tnow)
{

    double max_distance = interchange_radius_ * 2.0;
    std::vector<MapNodePtr> close_nodes = GetClosestNodes(Tnow, max_distance);
    MapNodePtr map_closest_observation = NULL;
    double min_distance = DBL_MAX;
    for (std::vector<MapNodePtr>::iterator itr_node = close_nodes.begin();
         itr_node != close_nodes.end(); ++itr_node)
    { // loop thorugh all close nodes and find the observation closest to Tnow among all map nodes
        ndt_generic::Affine3dSTLVek obs_vec = (*itr_node)->GetObservationVector();
        double distance = ndt_generic::SearchForClosestElement(Tnow, obs_vec);
        if (distance < min_distance)
        {
            min_distance = distance;
            map_closest_observation = (*itr_node);
        }
    }
    return map_closest_observation;
}
MapNodePtr GraphMapNavigator::GetMapNodeByObservationHistory(const Eigen::Affine3d& Tnow,
                                                             unsigned int n_search)
{

    double max_distance = 100; // interchange_radius_*2.0;
    std::vector<MapNodePtr> close_nodes =
        GetClosestNodes(Tnow, max_distance); // get all nodes within a distance

    std::vector<std::pair<double, MapNodePtr>> observation_distance;

    for (std::vector<MapNodePtr>::iterator itr_node = close_nodes.begin();
         itr_node != close_nodes.end(); ++itr_node)
    { // loop thorugh all close nodes and find the observation closest to Tnow among all map nodes
        ndt_generic::Affine3dSTLVek obs_vec =
            (*itr_node)->GetObservationVector(); // get all observation locations in submap
        double distance;
        for (int i = 0; i < obs_vec.size(); i++)
        {
            distance = ndt_generic::GetDistance(
                obs_vec[i], Tnow); // calculate distance from robot sensor to each observation
            observation_distance.push_back(
                std::make_pair(distance, *itr_node)); // create toubles of (distance , map-node)
        }
    }
    // Sort vector by distance to observation
    std::sort(observation_distance.begin(), observation_distance.end()); // sort vector of toubles
    std::vector<unsigned int> obs_histogram(Size(),
                                            0); // histogram over nodes for n closest observations
    for (int i = 0; i < observation_distance.size() && i < n_search; i++)
    { // create histogram of map-node occurance, data for histogram is limited by close
      // nodes*n_search
        unsigned int node_id = observation_distance[i].second->GetId();
        obs_histogram[node_id] = obs_histogram[node_id] + 1;
    }
    unsigned int max_index = distance(
        obs_histogram.begin(),
        max_element(obs_histogram.begin(), obs_histogram.end())); // select most occuring node

    return GetNode(max_index);
}

MapNodePtr GraphMapNavigator::GetClosestMapNode(const Eigen::Affine3d& Tnow,
                                                const bool use_observation_centroid)
{
    MapNodePtr closest_map_node = NULL;
    double min_dist = DBL_MAX;
    for (std::vector<MapNodePtr>::iterator itr_node = map_nodes_.begin();
         itr_node != map_nodes_.end(); ++itr_node)
    { // loop thorugh all existing nodes, select the closest one
        int id = currentNode_->GetId();
        int oid = (*itr_node)->GetId();

        double distance =
            ndt_generic::GetDistance((*itr_node)->GetMapPose(use_observation_centroid), Tnow);

        distance += abs(id - oid);
        if (distance < min_dist)
        {
            closest_map_node = *itr_node;
            min_dist = distance;
        }
    }
    return closest_map_node;
}

bool GraphMapNavigator::SwitchToMapNode(MapNodePtr new_node)
{
    if (new_node != NULL && currentNode_ != new_node)
    {
        // note: loading
        if (new_node->isOnDisk())
        {
            bool success = new_node->loadFromDisk();
            if (!success)
            {
                std::cout << "No success, removing" << std::endl;
                RemoveNode(new_node);
                return false;
            }
        }
        prevNode_ = currentNode_;
        currentNode_ = new_node;
        int newId = new_node->GetId();
        int prevId = prevNode_->GetId();
        cout << "Switched to node: " << newId
             << ", at: " << currentNode_->GetPose().translation().transpose() << endl;

        if (NDTMapPtr mntp = boost::dynamic_pointer_cast<NDTMapType>(currentNode_->GetMap()))
        {
            NDTMap* ndt = mntp->GetNDTMap();
            NDTMapParamPtr nmpp = boost::dynamic_pointer_cast<NDTMapParam>(mapparam_);

            if(nmpp && ndt)
            {
                ndt->setParameters(nmpp->positive_update_static_, nmpp->negative_update_static_,
                                nmpp->eta_static_, nmpp->positive_update_dynamic_,
                                nmpp->negative_update_dynamic_, nmpp->eta_dynamic_, nmpp->w_own_,
                                nmpp->w_cluster_);
            }
        }

        if (map_unloading_)
        {
            for (MapNodePtr map : map_nodes_)
            {
                int id = map->GetId();
                if (id == newId)
                {
                    continue;
                }
                bool onDisk = map->isOnDisk();
                if (not onDisk)
                {
                    map->unloadToDisk(getMapFileID());
                }
            }
        }
        int val = visited_nodes_[newId];
        visited_nodes_[newId] = val + 1;
        previous_map_id_ = prevId;
        return true;
    }
    else
    {
        return false;
    }
}

void GraphMapNavigator::getNodeStates()
{
    std::vector<int> loaded;
    std::vector<int> unloaded;
    for (MapNodePtr map : map_nodes_)
    {
        int id = map->GetId();
        bool onDisk = map->isOnDisk();
        if (onDisk)
        {
            unloaded.push_back(id);
        }
        else
        {
            loaded.push_back(id);
        }
    }

    if (loaded.size() > 20)
    {
        std::cout << loaded.size() << " loaded nodes" << std::endl;
    }
    else
    {
        std::cout << "Loaded nodes: [";
        for (unsigned int i = 0; i < loaded.size(); i++)
        {
            std::cout << loaded[i];
            if (i != loaded.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }
    if (unloaded.size() > 20)
    {
        std::cout << unloaded.size() << " unloaded nodes" << std::endl;
    }
    else
    {
        std::cout << "Unloaded nodes: [";
        for (unsigned int i = 0; i < unloaded.size(); i++)
        {
            std::cout << unloaded[i];
            if (i != unloaded.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }
}

void GraphMapNavigator::unloadAllSubmaps()
{
    for (MapNodePtr map : map_nodes_)
    {
        bool onDisk = map->isOnDisk();
        if (not onDisk)
        {
            map->unloadToDisk(getMapFileID());
        }
    }
}

bool GraphMapNavigator::AutomaticMapInterchange(const Eigen::Affine3d& Tnow,
                                                const Matrix6d& cov_incr,
                                                bool& changed_map_node,
                                                bool& created_map_node)
{

    created_map_node = false;
    changed_map_node = false;
    static Matrix6d pose_covar = unit_covar;
    bool mapChange = false;
    if (use_submap_)
    {
        if (map_switch_method_ == node_position_esg)
        {
            mapChange = TransitionESG(Tnow, cov_incr, changed_map_node, created_map_node);
        }
        else
        {
            mapChange = TransitionSG(Tnow, cov_incr, changed_map_node, created_map_node);
        }
        if (mapChange)
        {
            getNodeStates();
            return mapChange;
        }
    }
    else
    {
        return false;
    }
}

bool GraphMapNavigator::TransitionSG(const Eigen::Affine3d& Tnow,
                                     const Matrix6d& cov_incr,
                                     bool& changed_map_node,
                                     bool& created_map_node)
{
    changed_map_node = SwitchToClosestMapNode(Tnow, interchange_radius_);
    double distance = ndt_generic::GetDistance(GetCurrentNodePose(), Tnow);

    // if no node already exists, create a new node
    if (distance > interchange_radius_)
    {
        cout << "No node was found, will create a new map pose." << endl;
        AddMapNode(GetCurrentNodePose().inverse() * Tnow, cov_incr);
        prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(), prevNode_->GetPose(),
                                                  currentNode_->GetPose(), compound_radius_);
        created_map_node = true;
    }
    return created_map_node || changed_map_node;
}

bool GraphMapNavigator::TransitionESG(const Eigen::Affine3d& Tnow,
                                      const Matrix6d& cov_incr,
                                      bool& changed_map_node,
                                      bool& created_map_node)
{
    created_map_node = changed_map_node = false;
    // No longer within radius of node
    if (!currentNode_->WithinRadius(Tnow, interchange_radius_))
    {
        AddMapNode(GetCurrentNodePose().inverse() * Tnow, cov_incr);
        prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(), prevNode_->GetPose(),
                                                  currentNode_->GetPose(), compound_radius_);
        created_map_node = true;
    }
    return created_map_node || changed_map_node;
}

void GraphMapNavigator::UpdateGraph(const Eigen::Affine3d& pose,
                                    pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    GetCurrentNode()->updateMap(pose, cloud);
}

string GraphMapNavigator::ToString()
{
    std::stringstream ss;
    ss << "GraphMapNavigator:\n" << GraphMap::ToString();
    ss << "Interchange: " << interchange_radius_ << endl;
    ss << "Compound_radius: " << compound_radius_ << endl;
    ss << "Use keyframe: " << std::boolalpha << use_keyframe_ << endl;
    if (use_keyframe_)
    {
        ss << "Keyframe distance [m]: " << min_keyframe_dist_ << endl;
        ss << "Keyframe angular distance [deg]: " << min_keyframe_rot_deg_ << endl;
    }
    ss << "Switch method: " << GraphMapNavigatorParam::SwitchMethod2String(map_switch_method_)
       << endl;
    return ss.str();
}
MapSwitchingMethod GraphMapNavigatorParam::String2SwitchMethod(const std::string& switch_method)
{

    if (switch_method == "mean_observation")
        return mean_observation;
    else if (switch_method == "closest_observation")
        return closest_observation;
    else if (switch_method == "grid")
        return grid;
    else if (switch_method == "overlap")
        return overlap;
    else if (switch_method == "overlap_registration")
        return overlap_registration;
    else if (switch_method == "node_position_esg")
        return node_position_esg;
    else
        return node_position;
}
std::string GraphMapNavigatorParam::SwitchMethod2String(const MapSwitchingMethod& switch_method)
{
    if (switch_method == mean_observation)
        return std::string("mean_observation");
    else if (switch_method == closest_observation)
        return std::string("closest_observation");
    else if (switch_method == grid)
        return std::string("grid");
    else if (switch_method == node_position_esg)
        return std::string("node_position_esg");
    else if (switch_method == overlap)
        return std::string("overlap");
    else if (switch_method == overlap_registration)
        return std::string("overlap_registration");
    else
        return std::string("node_position");
}

void GraphMapNavigatorParam::GetParametersFromRos()
{
    ros::NodeHandle nh("~");
    nh.param("use_submap", use_submap, false);
    nh.param("check_for_broken_submaps", check_for_broken_submaps, false);
    nh.param("interchange_radius", interchange_radius, 10.0);
    nh.param("compound_radius", compound_radius, 0.0);
    nh.param("use_keyframe", use_keyframe, true);
    nh.param("min_keyframe_dist", min_keyframe_dist, 0.5);
    nh.param("min_keyframe_rot_deg", min_keyframe_rot_deg, 5.0);
    std::string sw_method;
    nh.param<string>("map_switching_method", sw_method, "node_position");
    map_switch_method = String2SwitchMethod(sw_method);
}

bool LoadGraphMap(const std::string& file_path,
                  const std::string& file_name,
                  GraphMapNavigatorPtr& ptr,
                  bool check_for_broken_submaps)
{
    std::string completePath = file_path + file_name;
    if (file_name == "" || file_path == "")
    {
        cout << "No map file path provided" << endl;
        exit(0);
    }
    std::ifstream f(completePath.c_str());
    if (!f.good())
    {
        cout << "The file  \"" << completePath << "\" does not exist." << endl;
        exit(0);
    }

    try
    {
        cout << "Loading map at path: \n" << completePath << endl;
        std::ifstream ifs(completePath);
        boost::archive::binary_iarchive ia(ifs);
        ia& ptr;

        std::string id = ptr->getMapFileID();
        cout << "Map " << id << " loaded" << endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading map at path:\n" << completePath << std::endl;
        cerr << e.what() << endl;
        return false;
    }
    if (ptr == NULL)
    {
        std::cerr << "ERROR LOADING NDT MAP FROM FILE" << std::endl;
        return false;
    }

    ptr->setMapDirToAll(file_path);

    if (check_for_broken_submaps)
    {
        auto mapNodes = ptr->GetMapNodes();
        NodePtrVector nodes = ptr->GetNodes();
        std::cout << "Map has " << nodes.size() << " nodes and " << mapNodes.size() << " map nodes."
                  << std::endl;
        for (MapNodePtr node : mapNodes)
        {
            bool success = false;

            if (node->isOnDisk())
            {
                success = node->loadFromDisk();
                if (success)
                {
                    success = node->unloadToDisk();
                }
            }
            else
            {
                std::cout << "Node not on disk, but it should be, fixing..." << std::endl;
                std::cout << "load id: " << node->getUnloadFile() << std::endl;
                success = node->loadFromDisk();
                if (success)
                {
                    success = node->unloadToDisk();
                    std::cout << "fix complete!" << std::endl;
                }
            }

            if (success)
            {
                std::cout << "Node " << node->GetId() << " ok" << std::endl;
            }
            else
            {
                std::cout << "Node " << node->GetId() << " broken, removing" << std::endl;
                ptr->RemoveNode(node);
            }
        }

        nodes = ptr->GetNodes();
        if (nodes.size() > 0)
        {
            MapNodePtr first = ptr->GetNode(0);
            ptr->ForceSetCurrentNode(first);
            MapNodePtr last = ptr->GetNode(nodes.size() - 1);
            ptr->ForceSetPreviousNode(last);
        }
    }

    return true;
}
void SaveObservationVector(const std::string& file_name, GraphMapNavigatorPtr graph_map)
{
    std::ofstream observations_file;
    std::string name = file_name + std::string("_observation.txt");
    observations_file.open(name.c_str());
    if (observations_file.is_open())
    {
        for (std::vector<MapNodePtr>::iterator itr_node = graph_map->begin();
             itr_node != graph_map->end(); ++itr_node)
        { // loop thorugh all close nodes and find the observation closest to Tnow among all map
          // nodes
            ndt_generic::Affine3dSTLVek obs_vec = (*itr_node)->GetObservationVector();
            for (int i = 0; i < obs_vec.size(); i++)
            {
                observations_file << obs_vec[i].translation()(0) << " "
                                  << obs_vec[i].translation()(1) << " "
                                  << obs_vec[i].translation()(2) << " "
                                  << std::distance(graph_map->begin(), itr_node) << std::endl;
                observations_file.flush();
            }
        }
        observations_file.close();
    }
    else
    {
        std::cout << "Error creating evaluation output files at path:" << std::endl;
        std::cout << name << std::endl;
        exit(0);
    }
}
void SaveGraphMap(const std::string& path,
                  const std::string& filename,
                  GraphMapNavigatorPtr graph_map)
{
    graph_map->m_graph.lock();
    cout << "-----------------------------Saving---------------------------------\n"
         << graph_map->ToString() << endl;
    cout << "----------------------------------------------------------------------\nTo file path:"
         << filename << endl;
    std::ofstream ofs(path + filename);
    boost::archive::binary_oarchive ar(ofs);
    ar << graph_map;
    ofs.close();
    graph_map->m_graph.unlock();
}

} // namespace graph_map
} // namespace perception_oru
