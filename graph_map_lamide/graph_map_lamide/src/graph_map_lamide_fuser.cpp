#include "graph_map_lamide/graph_map_fuser.h"

namespace perception_oru
{
namespace graph_map
{

GraphMapFuser::GraphMapFuser(string maptype,
                             string registratorType,
                             const Eigen::Affine3d& init_pose,
                             const Eigen::Affine3d& sensorPose,
                             const std::string& path,
                             bool ndt_om,
                             bool only_static)
{
    graph_param_ = GraphMapNavigatorParamPtr(new GraphMapNavigatorParam());
    graph_param_->GetParametersFromRos();
    regParam_ = GraphFactory::CreateRegParam(registratorType);
    cout << "started reading reg par from ros" << endl;
    regParam_->GetParametersFromRos();
    regParam_->sensor_pose = sensorPose;
    sensorPose_ = sensorPose;
    cout << "finished reading reg par from ros" << endl;
    mapParam_ = GraphFactory::CreateMapParam(maptype);
    cout << "started reading map par from ros" << endl;
    mapParam_->GetParametersFromRos();
    cout << "time to create graph inside fuser" << endl;

    graph_map_ = GraphMapNavigatorPtr(new GraphMapNavigator(init_pose, mapParam_, graph_param_));
    registrator_ = GraphFactory::CreateRegistrationType(regParam_);
    use_keyframe_ = graph_param_->use_keyframe;
    min_keyframe_dist_ = graph_param_->min_keyframe_dist;
    min_keyframe_rot_deg_ = graph_param_->min_keyframe_rot_deg;
    nr_frames_ = 0;
    pose_last_fuse_ = init_pose;
    initialized_ = true;
    ndt_om_ = ndt_om;
    graph_map_->setMapDirToAll(path);
    only_static_ = only_static;
    is_lamide_ = registratorType == ndt_lamide_reg_type_name;
}
GraphMapFuser::GraphMapFuser(RegParamPtr regParam,
                             MapParamPtr mapParam,
                             GraphMapNavigatorParamPtr graph_param,
                             const Eigen::Affine3d& init_pose,
                             const Eigen::Affine3d& sensorPose,
                             const std::string& path,
                             bool ndt_om,
                             bool only_static)
{
    mapParam_ = mapParam;
    regParam_ = regParam;
    graph_param_ = graph_param;
    sensorPose_ = sensorPose;
    Eigen::Affine3d Tinit = init_pose;

    graph_map_ = GraphMapNavigatorPtr(new GraphMapNavigator(Tinit, mapParam_, graph_param_));
    registrator_ = GraphFactory::CreateRegistrationType(regParam_);

    nr_frames_ = 0;
    use_keyframe_ = graph_param_->use_keyframe;
    min_keyframe_dist_ = graph_param_->min_keyframe_dist;
    min_keyframe_rot_deg_ = graph_param_->min_keyframe_rot_deg;
    pose_last_fuse_ = init_pose;
    initialized_ = true;
    ndt_om_ = ndt_om;
    graph_map_->setMapDirToAll(path);
    only_static_ = only_static;
}

void GraphMapFuser::SaveGraphMap(const std::string& path, const std::string& filename, const std::string& prefix)
{
    std::string id = graph_map_->getMapFileID();

    // if the path is missing slash, add it
    std::string fixedpath = path;
    if (path[path.size()] != '/')
    {
        fixedpath = path + "/";
    }

    // everything is unloaded
    for (MapNodePtr node : graph_map_->GetMapNodes())
    {
        if (!node->isOnDisk())
        {
            node->unloadToDisk(id);
        }
    }

    std::string completePath = fixedpath + prefix + "_" + id + "_" + filename;

    // the structure is saved as the main map
    graph_map_->m_graph.lock();
    cout << "-----------------------------Saving---------------------------------\n"
         << graph_map_->ToString() << endl;
    cout << "----------------------------------------------------------------------\nTo file path:"
         << completePath << endl;
    std::ofstream ofs(completePath);
    boost::archive::binary_oarchive ar(ofs);
    ar << graph_map_;
    ofs.close();
    graph_map_->m_graph.unlock();
}

void GraphMapFuser::SavePointCloud(const std::string& path, const std::string& filename)
{
    SaveGraphMapPCD(path, "full_cloud.pcd", graph_map_);
}

void GraphMapFuser::SaveCurrentNodeAsJFF(const std::string& filename)
{
    graph_map_->m_graph.lock();
    cout << "-----------------------------Saving JFF---------------------------------\n"
         << graph_map_->ToString() << endl;
    cout << "----------------------------------------------------------------------\nTo file path:"
         << filename << endl;
    NDTMap* map =
        boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode())->GetNDTMap();
    std::string name = filename;
    map->writeToJFF(name.c_str());
    graph_map_->m_graph.unlock();
}

bool exists_test0(const std::string& name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

Eigen::MatrixXd GraphMapFuser::PredictOdomUncertainty(const Eigen::Affine3d Tnow, bool prediction2d)
{
    Eigen::Affine3d Tmotion = GetPoseLastFuse().inverse() * Tnow; // difference since last fuse
    Eigen::MatrixXd cov6d = motion_model_3d_.getCovMatrix(Tmotion);
    return cov6d;
}

void GraphMapFuser::AddPoseFramePair(const Eigen::Affine3d& Tnow, const Eigen::Affine3d& Tref)
{
    // Determine which node to add the poses to.
    MapNodePtr node_ptr = graph_map_->GetCurrentNode();
    if (node_ptr != NULL)
    {
        if (node_ptr->GetMap()->Initialized())
        {
            Eigen::Affine3d Tlocal = Tnow;
            graph_map_->WorldToLocalMapFrame(Tlocal, node_ptr); // Convert to local frame...
            node_ptr->AddPoseFramePair(Tlocal, Tref);
        }
    }
}

void GraphMapFuser::Visualize(bool enableVisualOutput, plotmarker marker)
{
    visualize_ = enableVisualOutput;
    marker_ = marker;
}

bool GraphMapFuser::FuseFrame(const Affine3d& Tnow, const Affine3d& Tmotion)
{
    if (fuse_no_motion_frames)
        return FuseNoMotionFrames(Tnow, Tmotion);
    else
        return KeyFrameBasedFuse(Tnow);
}

bool GraphMapFuser::KeyFrameBasedFuse(const Affine3d& Tnow)
{

    bool ret = false;
    static int frame_idx = 0;
    if (frame_idx == 0)
    {
        frame_idx++;
        return true;
    }
    else
    {
        frame_idx++;
        Affine3d diff = pose_last_fuse_.inverse() * Tnow;
        Eigen::Vector3d Tmotion_euler = diff.rotation().eulerAngles(0, 1, 2);
        ndt_generic::normalizeEulerAngles(Tmotion_euler);

        // cout<<"diff
        // (trans[m]/rot[deg])=("<<diff.translation().norm()<<"/"<<diff.rotation().eulerAngles(0,1,2).norm()*180.0/M_PI<<")
        // limit=("<<min_keyframe_dist_<<"/"<<min_keyframe_rot_deg_<<")"<<endl;
        if (use_keyframe_)
        {
            if (diff.translation().norm() > min_keyframe_dist_ ||
                Tmotion_euler.norm() > (min_keyframe_rot_deg_ * M_PI / 180.0))
            {
                ret = true;
            }
            else
            {
                ret = false;
            }
        }
        else
        {
            ret = true;
        }
    }
    frame_idx++;
    if (ret == true && avoid_lidar_shadow)
    {
        if (frame_idx % 4 == 0)
        { // Avoid that the scanner covers the same region (one scan covers ~270 degrees angle and
          // the same area is repeaded every 4rd frame)
            ret = false;
        }
        else
        {
            frame_idx = 0;
        }
    }
    return ret;
}

bool GraphMapFuser::FuseNoMotionFrames(const Affine3d& Tnow, const Affine3d& Tmotion)
{
    const double no_mot_t = 0.01, no_mot_r = M_PI / 180.0; // 1cm /s resp. 1 deg/s
    const double min_dist = 1.5;
    static unsigned int fused_frames = 0;
    Vector3d euler = Tmotion.linear().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(euler);

    Eigen::Affine3d diff;
    diff = pose_last_fuse_.inverse() * Tnow;
    if (Tmotion.translation().norm() * 13 < no_mot_t && euler.norm() * 13.0 < no_mot_r &&
        diff.translation().norm() > min_dist)
    {
        if (fused_frames < 100)
        {
            cout << "FUSE frame=" << fused_frames
                 << ", t_vel=" << Tmotion.translation().norm() / 13.0
                 << " m/s , rot_vel=" << euler.norm() << endl;
            fused_frames++;
            return true;
        }
    }
    else
    {
        fused_frames = 0;
    }
    return false;
}

void GraphMapFuser::SetFuserOptions(bool save_merged_cloud)
{
    save_merged_cloud_ = save_merged_cloud;
}
MapTypePtr GraphMapFuser::CreateMap(
    std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<pcl::PointXYZL>>>& reference)
{
    Eigen::Affine3d Tframe;
    if (reference.size() == 0)
    {
        Tframe = Eigen::Affine3d::Identity();
        return NULL;
    }
    /*else
      Tframe = reference.end()->first.inverse();*/
    MapTypePtr ptr = GraphFactory::CreateMapType(mapParam_);
    pcl::PointCloud<pcl::PointXYZL> pmerged;
    for (auto p : reference)
        ptr->updateMap(reference.end()->first * sensorPose_, p.second, true);

    std::cout << "after: " << ptr->Initialized() << std::endl;
    // pmerged+=p.second;
    // transformPointCloudInPlace(Tframe, pmerged);

    cout << "Create map" << endl;
    return ptr;
}

bool GraphMapFuser::scanmatching(
    std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<pcl::PointXYZL>>>& reference,
    pcl::PointCloud<pcl::PointXYZL>& source,
    Eigen::Affine3d& Todom,
    Eigen::Affine3d& Tcorrected,
    Eigen::MatrixXd& reg_cov,
    size_t submap_size)
{
    MapTypePtr map;
    static bool first = true;
    static Eigen::Affine3d Todomprevfused = Todom;
    if (first)
    {
        pose_last_fuse_ = Todom;
        Eigen::Affine3d Tsensinworld = pose_last_fuse_ * sensorPose_;
        Tcorrected = Todom;
        transformPointCloudInPlace(Tsensinworld, source); // transform to robot odometry frame
        reference.push_back(std::make_pair(pose_last_fuse_, source));
        first = false;
        return true;
    }
    Eigen::Affine3d Tmotion = Todomprevfused.inverse() * Todom;
    Eigen::MatrixXd Cov = motion_model_3d_.getCovMatrix(Tmotion);

    bool fuse_this_frame =
        KeyFrameBasedFuse(pose_last_fuse_ * Tmotion); // fuse frame based on distance traveled
    if (!fuse_this_frame)
    {
        return false;
    }

    map = CreateMap(reference);
    Eigen::Affine3d Tlast = pose_last_fuse_.inverse();
    transformPointCloudInPlace(sensorPose_, source); // transform to robot odometry frame
    // pcl::PointCloud<pcl::PointXYZL> lsource = source;
    // transformPointCloudInPlace(Tlast, lsource);//register in local frame
    cout << "registed with cov:\n" << Cov << endl;
    Tcorrected = pose_last_fuse_ * Tmotion;
    bool registration_succesfull = registrator_->RegisterScan(
        map, Tcorrected, source, Cov); // Tcorrected will be updated to the actual pose of the robot
                                       // according to ndt-d2d registration

    if (registration_succesfull)
    {
        if (registrator_->calculate_cov_ && registrator_->enable_registraiton)
            reg_cov = registrator_->OutputCov;
        else
            reg_cov = unit_covar;
        transformPointCloudInPlace(Tcorrected, source);
        reference.push_back(std::make_pair(Tcorrected, source));
        pose_last_fuse_ = Tcorrected;
        Todomprevfused = Todom;
        if (reference.size() > submap_size)
            reference.erase(reference.begin());
        return true;
    }
    else
        reg_cov = unit_covar;
    return false;
}

void GraphMapFuser::PlotMapType(MapTypePtr map)
{

    GraphPlot::PlotMap(map, -1, Eigen::Affine3d::Identity());
    // GraphPlot::PlotPoseGraph(graph_map_);
}

bool GraphMapFuser::ErrorStatus(string status)
{
    if (graph_map_ != NULL && registrator_ != NULL)
    {
        return false;
    }
    else
    {
        status = "No object instance found for graph or registrator";
        return true;
    }
}
// void GetCovarianceFromMotion(Matrix6d &cov,const Affine3d &Tm){
// }
std::string GraphMapFuser::ToString()
{
    std::stringstream ss;
    ss << "----------------------fuser parameters--------------------------" << endl;
    ss << "fuser:" << endl << "initialized:" << initialized_ << endl;
    ss << "visualize:" << visualize_ << endl;
    ss << "Key frame based update:" << std::boolalpha << use_keyframe_ << endl;
    if (use_keyframe_)
        ss << "minimum keyframe distance(meter/deg):(" << min_keyframe_dist_ << ","
           << min_keyframe_rot_deg_ << ")" << endl;

    ss << registrator_->ToString();
    ss << graph_map_->ToString();
    ss << "----------------------fuser parameters--------------------------" << endl;
    return ss.str();
}

// ██████╗ ███████╗██████╗ ██╗   ██╗ ██████╗
// ██╔══██╗██╔════╝██╔══██╗██║   ██║██╔════╝
// ██║  ██║█████╗  ██████╔╝██║   ██║██║  ███╗
// ██║  ██║██╔══╝  ██╔══██╗██║   ██║██║   ██║
// ██████╔╝███████╗██████╔╝╚██████╔╝╚██████╔╝
// ╚═════╝ ╚══════╝╚═════╝  ╚═════╝  ╚═════╝

void checkLabels(const std::vector<perception_oru::NDTCell*>& ndts)
{
    int empty = 0;
    int nonEmpty = 0;
    int total = ndts.size();
    for (unsigned int i = 0; i < total; i++)
    {
        perception_oru::NDTCell* cell = ndts[i];
        int label = cell->getLabel();
        int weight = cell->getLabelWeight();

        if (weight == 0)
        {
            empty++;
        }
        else
        {
            nonEmpty++;
        }
    }
    std::cout << "    empty labels: " << empty << std::endl;
    std::cout << "non-empty labels: " << nonEmpty << std::endl;
    std::cout << "    total  cells: " << total << std::endl;
    std::cout << std::endl;
}

void GraphMapFuser::checkLabelConsistency(const pcl::PointCloud<pcl::PointXYZL>& registered_cloud,
                                          const Eigen::Affine3d& pose)
{
    boost::shared_ptr<NDTMapType> current_map =
        boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
    std::string frame = GetNodeLinkName(graph_map_->GetCurrentNode());
    double res = current_map->GetResolution();
    perception_oru::NDTMap local_map(new perception_oru::LazyGrid(res));

    local_map.loadPointCloud(registered_cloud);
    local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    std::vector<perception_oru::NDTCell*> ndts_map = current_map->GetNDTMap()->getAllCellsNoCopy();
    std::vector<perception_oru::NDTCell*> ndts_local = local_map.getAllCellsNoCopy();
    std::cout << "MAP" << std::endl;
    checkLabels(ndts_map);
    std::cout << "Local map" << std::endl;
    checkLabels(ndts_local);
}

void GraphMapFuser::plotErrorMaps(const pcl::PointCloud<pcl::PointXYZL>& registered_cloud,
                                  const Eigen::Affine3d& pose)
{
    boost::shared_ptr<NDTMapType> current_map =
        boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
    std::string frame = GetNodeLinkName(graph_map_->GetCurrentNode());
    double res = current_map->GetResolution();
    perception_oru::NDTMap local_map(new perception_oru::LazyGrid(res));

    local_map.loadPointCloud(registered_cloud);
    local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    // These are for custom coloring of voxels
    std::vector<perception_oru::NDTCell*> ndts_original = local_map.getAllCellsNoCopy();
    // ColorVoxelsWithScore(ndts_original, pose);
    ColorVoxelsWithLabel(ndts_original);

    perception_oru::NDTMap* mapPtr = &local_map;
    Eigen::Affine3d I = Eigen::Affine3d::Identity();
    GraphPlot::PlotNDTRGB(mapPtr, frame, "local_map", pose);
    delete mapPtr;
}

void GraphMapFuser::ColorVoxelsWithScore(std::vector<perception_oru::NDTCell*>& ndts,
                                         const Eigen::Affine3d& pose)
{
    // ranging
    boost::shared_ptr<NDTMapType> current_map =
        boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
    NDTMap* map = current_map->GetNDTMap();

    double maxScore = std::numeric_limits<double>::min();
    double minScore = std::numeric_limits<double>::max();
    std::map<int, double> scores;
    std::vector<double> scoresList;

    int Nn = 0;

    unsigned int nr_cells = 0;
    // this was particle pose
    Eigen::Affine3d T = pose;

    double score = 1;

    if (ndts.size() == 0)
        fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
    Nn = ndts.size();

    for (int n = 0; n < ndts.size(); n++)
    {
        Eigen::Vector3d m = T * ndts[n]->getMean();

        perception_oru::NDTCell* cell;
        pcl::PointXYZL p;
        p.x = m[0];
        p.y = m[1];
        p.z = m[2];

        if (map->getCellAtPoint(p, cell))
        {
            // if(map.getCellForPoint(p,cell)){
            if (cell == NULL)
            {
                scores[n] = -1;
                continue;
            }
            if (cell->hasGaussian_)
            {
                // This scope calculates the L2 distance in parts

                // First the combined covariance
                Eigen::Matrix3d covCombined =
                    cell->getCov() + T.rotation() * ndts[n]->getCov() * T.rotation().transpose();
                Eigen::Matrix3d icov;
                bool exists;
                double det = 0;

                // Get the inverse convariance
                covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                if (!exists)
                {
                    scores[n] = -1;
                    continue;
                }

                // This calculates the likelihood
                double l = (cell->getMean() - m)
                               .dot(icov * (cell->getMean() - m)); //(cell->getMean()-m).norm()
                if (l * 0 != 0)
                {
                    scores[n] = -1;
                    continue;
                }

                // And this applies the scaling parameters
                // d1 = score_cell_weight + (1.0 - score_cell_weight)
                // d2 => * exp(-d2*l/2.0)
                double d1 = 1;
                double d2 = 0.05;
                double voxelScore = d1 * exp(-d2 * l / 2.0);
                score += voxelScore;

                // ranging
                if (voxelScore > maxScore)
                {
                    maxScore = voxelScore;
                }
                if (voxelScore < minScore)
                {
                    minScore = voxelScore;
                }
                scores[n] = voxelScore;
                scoresList.push_back(voxelScore);
                nr_cells++;
            }
        }
    }

    // properties
    double sum = std::accumulate(scoresList.begin(), scoresList.end(), 0.0);
    double mean = sum / scoresList.size();

    double sq_sum =
        std::inner_product(scoresList.begin(), scoresList.end(), scoresList.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / scoresList.size() - mean * mean);

    std::cout << "# points: " << scoresList.size() << std::endl;
    std::cout << "min: " << minScore << " mean: " << mean << " max: " << maxScore << std::endl;
    std::cout << "stdev: " << stdev << std::endl;

    // coloring
    for (int n = 0; n < ndts.size(); n++)
    {
        double score = scores[n];
        NDTCell* cell = ndts[n];

        // double normalized = (score - minScore) / maxScore;
        //  float r = fmin(2 * normalized, 1.0f);
        //  float g = fmin(2 * (1 - normalized), 1.0f);
        float r = 0;
        float g = 0;
        float b = 0;
        if (score < 0)
        {
            b = 1.0;
        }
        else
        {
            r = fmin(4 * (1 - score), 1.0f);
            g = fmin(4 * score, 1.0f);
        }

        // cell->setRGB(r, g, b);
    }
}

void GraphMapFuser::ColorVoxelsWithLabel(std::vector<perception_oru::NDTCell*>& ndts)
{
    int nonzero = 0;
    int notfound = 0;
    for (perception_oru::NDTCell* cell : ndts)
    {
        int label = cell->getLabel();

        nonzero = label != 0 ? nonzero + 1 : nonzero;

        auto it = ndt_generic::color_map_.find(label);
        if (it != ndt_generic::color_map_.end())
        {
            std::vector<int> color = ndt_generic::color_map_.at(label);
            float r = color[0];
            float g = color[1];
            float b = color[2];
            // cell->setRGB(r, g, b);
            // std::cout << "label: " << label << " to: " << r << " " << g << " " << b << std::endl;
        }
        else
        {
            // std::cout << "didn't find label: " << label << std::endl;
            // cell->setRGB(1.0f, 1.0f, 1.0f);
            notfound++;
        }
    }
    // std::cout << "out of " << ndts.size() << " ndts" << std::endl;
    // std::cout << "colored " << nonzero << " nonzero labels" << std::endl;
    // std::cout << notfound << " labels not found" << std::endl;
}

void GraphMapFuser::setUseOdomAsGT(bool use)
{
    use_odom_as_gt_ = use;
}

} // namespace graph_map
} // namespace perception_oru
