#include <ndt_generic_lamide/utils.h>

namespace perception_oru
{
namespace graph_map
{

template <class PointT>
bool GraphMapFuser::ProcessFrame(pcl::PointCloud<PointT>& cloud,
                                 Eigen::Affine3d& Tnow,
                                 Eigen::Affine3d& Tmotion)
{
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6);
    return ProcessFrame(cloud, Tnow, Tmotion, cov);
}

// ███████╗██████╗  █████╗ ███╗   ███╗███████╗
// ██╔════╝██╔══██╗██╔══██╗████╗ ████║██╔════╝
// █████╗  ██████╔╝███████║██╔████╔██║█████╗
// ██╔══╝  ██╔══██╗██╔══██║██║╚██╔╝██║██╔══╝
// ██║     ██║  ██║██║  ██║██║ ╚═╝ ██║███████╗
// ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝

/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */
template <class PointT>
bool GraphMapFuser::ProcessFrame(pcl::PointCloud<PointT>& cloud,
                                 Eigen::Affine3d& Tnow,
                                 Eigen::Affine3d& Tmotion,
                                 Eigen::MatrixXd& Tcov)
{
    // init
    if (!initialized_)
    {
        cerr << "fuser not initialized" << endl;
        return false;
    }

    // vars
    static bool map_node_changed = false;
    bool registration_succesfull = true;
    bool map_node_created = false, fuse_this_frame = false;
    Tnow = Tnow * Tmotion;

    //
    // ─── CHECK FUSING ───────────────────────────────────────────────────────────────
    //

    // FIXME: reafactor out
    fuse_this_frame = true;

    //
    // ─── TRANSFORM ──────────────────────────────────────────────────────────────────
    //

    graph_map_->m_graph.lock();
    // Transform cloud into robot frame before registrating
    if (fuse_this_frame || map_node_changed)
    {
        if (graph_map_->GetCurrentMapNode()->Initialized())
        {
            //default case
            if (!use_odom_as_gt_)
            {
                graph_map_->WorldToLocalMapFrame(Tnow);
                // Tnow will be updated to the actual pose of the
                // robot according to ndt-d2d registration
                registration_succesfull = registrator_->RegisterScan(
                    graph_map_->GetCurrentMapNode(), Tnow, cloud, Tcov);
                graph_map_->LocalToWorldMapFrame(Tnow);
            }
            //! this is for debugging. treat odom as GT (as it is in Kitti)
            else
            {
                //std::cout << "Use odom as GT" << std::endl;
                registration_succesfull = true;
            }
        }
        Eigen::Affine3d Tsensor_pose;
        Tsensor_pose = Tnow * sensorPose_;
        graph_map_->AutomaticMapInterchange(Tsensor_pose, Tcov, map_node_changed, map_node_created);
    }

    //
    // ─── UPDATE ─────────────────────────────────────────────────────────────────────
    //

    //static maps only!
    // note: if LaMiDE registration, do not put non-static points into map
    if (is_lamide_ and only_static_)
    {
        //std::cout << "cloud size before: " << cloud.points.size() << std::endl;
        std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>> filtered;
        filtered.resize(cloud.points.size());
        int idx = 0;
        for (unsigned int i = 0; i < cloud.points.size(); i++)
        {
            if (std::find(ndt_generic::static_labels_.begin(), ndt_generic::static_labels_.end(), cloud.points[i].label) !=
                ndt_generic::static_labels_.end())
            {
                filtered[idx] = cloud.points[i];
                idx++;
            }
        }
        filtered.resize(idx + 1);
        cloud.points.swap(filtered);
        //std::cout << "cloud size after: " << cloud.points.size() << std::endl;
    }

    if ((fuse_this_frame && registration_succesfull) || map_node_created)
    { // if changed to previously created map node, start by map_registration
        if (multiple_map_update && graph_map_->Size() > 1)
        {
            UpdateMultipleMaps(cloud, Tnow);
        }
        else
        {
            UpdateSingleMap(cloud, Tnow);
        }

        graph_map_->m_graph.unlock();
        nr_frames_++;
        return true;
    }
    else
    {
        graph_map_->m_graph.unlock();
        return false;
    }
}

template <class PointT>
bool GraphMapFuser::ProcessFrameStaticGlobalMap(pcl::PointCloud<PointT>& cloud,
                                                Eigen::Affine3d& Tnow,
                                                Eigen::Affine3d& Tmotion)
{

    static bool fuse_prev = true;
    bool fuse_frame = true;
    static pcl::PointCloud<PointT> merged_cloud;
    static unsigned int merged_clouds_count = 0;
    bool registration_succesfull = true;
    bool updated = false;
    static int stand_still_frames = 0;
    static int nb_frames = 0;
    if (!initialized_)
    {
        cerr << "fuser not initialized" << endl;
        return false;
    }

    nr_frames_++;
    Tnow = Tnow * Tmotion;

    fuse_frame = FuseFrame(Tnow, Tmotion); // return true if vehicle standing still
    cout << "fuse?: " << fuse_frame << endl;
    if (fuse_frame == true && fuse_prev == false)
    { // stannade precis till
        transformPointCloudInPlace(sensorPose_, cloud);
        merged_cloud += cloud;
        merged_clouds_count++;
    }
    else if (fuse_frame == true && fuse_prev == true)
    { // been standing still since last frame
        transformPointCloudInPlace(sensorPose_, cloud);
        merged_cloud += cloud;
        merged_clouds_count++;
    }
    else if (fuse_frame == false && fuse_prev == true)
    { // started moving, should register towards previous map here

        Eigen::Affine3d Tinit = Tnow;
        if (graph_map_->GetCurrentMapNode()->Initialized())
        {
            graph_map_->WorldToLocalMapFrame(Tinit);
            registration_succesfull = registrator_->RegisterScan(
                graph_map_->GetCurrentMapNode(), Tinit,
                merged_cloud); // Tnow will be updated to the actual pose of the robot according to
                               // ndt-d2d registration
            graph_map_->LocalToWorldMapFrame(Tinit);
        }
        if (registration_succesfull)
        {
            Tnow = Tinit;
            updated = true;
            UpdateSingleMap(merged_cloud, Tnow);
            pose_last_fuse_ = Tnow;

            if (save_merged_cloud_)
            {
                Eigen::Affine3d sensorPose_inv = sensorPose_.inverse();
                transformPointCloudInPlace(sensorPose_inv, merged_cloud);
                pcl::io::savePCDFileASCII(
                    "merged_cloud" + ndt_generic::toString(nb_frames++) + ".pcd", merged_cloud);
            }
        }
        else
            cout << "REG error" << endl;

        merged_cloud.clear();
        merged_clouds_count = 0;
    }
    fuse_prev = fuse_frame;

    return updated;
}
template <class PointT>
bool GraphMapFuser::ProcessFrameStatic(pcl::PointCloud<PointT>& cloud,
                                       Eigen::Affine3d& Tnow,
                                       Eigen::Affine3d& Tmotion)
{

    static bool fuse_prev = true;
    bool fuse_frame = true;
    static pcl::PointCloud<PointT> merged_cloud;
    static unsigned int merged_clouds_count = 0;
    bool registration_succesfull = false;
    bool updated = false;
    if (!initialized_)
    {
        cerr << "fuser not initialized" << endl;
        return false;
    }

    nr_frames_++;
    Tnow = Tnow * Tmotion;

    fuse_frame = FuseFrame(Tnow, Tmotion); // return true if vehicle standing still
    if (fuse_frame == true && fuse_prev == false)
    { // stannade precis till
        transformPointCloudInPlace(sensorPose_, cloud);
        // graph_map_->AddMapNode(graph_map_->GetCurrentNodePose().inverse()*Tnow*sensorPose_);
        // //add new node of current node pose

        merged_cloud += cloud;
        merged_clouds_count++;
    }
    else if (fuse_frame == true && fuse_prev == true)
    { // been standing still since last frame
        transformPointCloudInPlace(sensorPose_, cloud);
        merged_cloud += cloud;
        merged_clouds_count++;
    }
    else if (fuse_frame == false && fuse_prev == true)
    { // started moving, should register towards previous map here
        UpdateSingleMap(merged_cloud, Tnow);
        pose_last_fuse_ = Tnow;
        merged_cloud.clear();
        merged_clouds_count = 0;

        if (graph_map_->Size() >= 2)
        {

            Eigen::Affine3d diff =
                graph_map_->GetPreviousNodePose().inverse() * graph_map_->GetCurrentNodePose();

            double score;
            cout << "register" << endl;
            NDTD2DRegTypePtr ndt_reg = boost::dynamic_pointer_cast<NDTD2DRegType>(registrator_);
            bool registration_succesfull; // register here

            if (registration_succesfull)
            {

                updated = true;
                cout << "REG success" << endl;
                Tnow = graph_map_->GetPreviousNodePose() * diff * sensorPose_.inverse();
                // NDTMapPtr prev_node = boost::dynamic_pointer_cast< NDTMapType
                // >(graph_map_->GetPreviousNode()->GetMap());
                //  GraphPlot::SendGlobalMapToRviz(prev_node->GetNDTMap(),1,graph_map_->GetPreviousNodePose());
                graph_map_->UpdateLink(graph_map_->GetPreviousNode(), graph_map_->GetCurrentNode(),
                                       graph_map_->GetPreviousNodePose().inverse() *
                                           graph_map_->GetCurrentNodePose());
                // GraphPlot::PlotMap(graph_map_->GetCurrentMapNode(),1,graph_map_->GetCurrentNodePose());
            }
            else
                cout << "REG error" << endl;
        }
    }
    else if (fuse_frame == false && fuse_prev == false)
    { // moving, do nothing
    }

    fuse_prev = fuse_frame;

    return updated;
}

template <class PointT>
void GraphMapFuser::UpdateSingleMap(pcl::PointCloud<PointT>& cloud, Eigen::Affine3d& Tnow)
{ // specified in the global frame
    // cout<<"Update"<<endl;
    graph_map_->WorldToLocalMapFrame(Tnow);
    transformPointCloudInPlace(Tnow, cloud);
    // Update map, provided transform is the pose of the sensor in
    // the world which is where the scan was taken from

    //! simple registration is NDT without OM capabilities
    bool simple = !ndt_om_;
    graph_map_->GetCurrentNode()->updateMap(Tnow * sensorPose_, cloud, simple, clusterUpdate_);
    graph_map_->LocalToWorldMapFrame(Tnow);
    pose_last_fuse_ = Tnow;
}

template <class PointT>
void GraphMapFuser::UpdateMultipleMaps(pcl::PointCloud<PointT>& cloud, Eigen::Affine3d& Tnow)
{ // specified in the global frame

    std::vector<MapNodePtr> closest_nodes =
        graph_map_->GetClosestNodes(Tnow * sensorPose_, 1.0, true);
    if (closest_nodes.size() == 0)
        cout << "close nodes There should always be a node within interchange radius" << endl;
    else if (closest_nodes.size() == 1)
    {
        UpdateSingleMap(cloud, Tnow);
        cout << "close nodes, one" << endl;
    }
    else
    {
        for (int i = 0; i < closest_nodes.size(); i++)
        {
            pcl::PointCloud<PointT> cloud_map_i;
            copyPointCloud(cloud_map_i, cloud);
            MapNodePtr map_i = closest_nodes[i];
            Eigen::Affine3d Tinit = Tnow;
            graph_map_->WorldToLocalMapFrame(Tinit, map_i);
            transformPointCloudInPlace(Tinit, cloud_map_i);
            // Update map, provided transform is the pose of the sensor in the
            // world which is where the scan was taken from

            //! simple registration is NDT without OM capabilities
            bool simple = !ndt_om_;
            map_i->updateMap(Tinit * sensorPose_, cloud_map_i, simple);

            cout << "update node=" << map_i->GetPose().translation() << endl;
        }
        pose_last_fuse_ = Tnow;
    }
}

//---------------------------------------------------------------------------------------------------

template <class PointT>
bool GraphMapFuser::ProcessFrame(std::vector<pcl::PointCloud<PointT>>& clouds,
                                 Eigen::Affine3d& Tnow,
                                 Eigen::Affine3d& Tmotion)
{
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6);
    return ProcessFrame(clouds, Tnow, Tmotion, cov);
}
/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */
template <class PointT>
bool GraphMapFuser::ProcessFrame(std::vector<pcl::PointCloud<PointT>>& clouds,
                                 Eigen::Affine3d& Tnow,
                                 Eigen::Affine3d& Tmotion,
                                 Eigen::MatrixXd& Tcov)
{

    if (!initialized_)
    {
        cerr << "fuser not initialized" << endl;
        return false;
    }

    static bool map_node_changed = false;
    bool registration_succesfull = true;
    bool map_node_created = false, fuse_this_frame = false;

    Tnow = Tnow * Tmotion;

    fuse_this_frame = FuseFrame(Tnow, Tmotion) |
                      map_node_changed; // fuse frame based on distance traveled, also fuse this
                                        // frame if just changed to precious map*/

    graph_map_->m_graph.lock();
    // Transform cloud into robot frame before registrating
    if (fuse_this_frame || map_node_changed)
    {
        if (graph_map_->GetCurrentMapNode()->Initialized())
        {
            graph_map_->WorldToLocalMapFrame(Tnow);
            registration_succesfull =
                registrator_->RegisterScan(graph_map_->GetCurrentMapNode(), Tnow, clouds,
                                           Tcov); // Tnow will be updated to the actual pose of the
                                                  // robot according to ndt-d2d registration
            graph_map_->LocalToWorldMapFrame(Tnow);
        }
        Eigen::Affine3d Tsensor_pose;
        Tsensor_pose = Tnow * sensorPose_;
        graph_map_->AutomaticMapInterchange(Tsensor_pose, Tcov, map_node_changed, map_node_created);
    }

    if ((fuse_this_frame && registration_succesfull) || map_node_created)
    { // if changed to previously created map node, start by map_registration
        if (multiple_map_update && graph_map_->Size() > 1)
        {
            UpdateMultipleMaps(clouds, Tnow);
        }
        else
        {
            UpdateSingleMap(clouds, Tnow);
        }

        graph_map_->m_graph.unlock();
        nr_frames_++;
        return true;
    }
    else
    {
        graph_map_->m_graph.unlock();
        return false;
    }
}

template <class PointT>
bool GraphMapFuser::ProcessFrameStaticGlobalMap(std::vector<pcl::PointCloud<PointT>>& clouds,
                                                Eigen::Affine3d& Tnow,
                                                Eigen::Affine3d& Tmotion)
{

    static bool fuse_prev = true;
    bool fuse_frame = true;
    static std::vector<pcl::PointCloud<PointT>> merged_clouds(clouds.size());
    static unsigned int merged_clouds_count = 0;
    bool registration_succesfull = true;
    bool updated = false;
    static int stand_still_frames = 0;
    static int nb_frames = 0;
    if (!initialized_)
    {
        cerr << "fuser not initialized" << endl;
        return false;
    }

    nr_frames_++;
    Tnow = Tnow * Tmotion;

    fuse_frame = FuseFrame(Tnow, Tmotion); // return true if vehicle standing still
    cout << "fuse?: " << fuse_frame << endl;
    if (fuse_frame == true && fuse_prev == false)
    { // stannade precis till
        for (size_t i = 0; i < clouds.size(); i++)
        {
            transformPointCloudInPlace(sensorPose_, clouds[i]);
            merged_clouds[i] += clouds[i];
        }
        merged_clouds_count++;
    }
    else if (fuse_frame == true && fuse_prev == true)
    { // been standing still since last frame
        for (size_t i = 0; i < clouds.size(); i++)
        {
            transformPointCloudInPlace(sensorPose_, clouds[i]);
            merged_clouds[i] += clouds[i];
        }
        merged_clouds_count++;
    }
    else if (fuse_frame == false && fuse_prev == true)
    { // started moving, should register towards previous map here

        Eigen::Affine3d Tinit = Tnow;
        if (graph_map_->GetCurrentMapNode()->Initialized())
        {
            graph_map_->WorldToLocalMapFrame(Tinit);
            registration_succesfull = registrator_->RegisterScan(
                graph_map_->GetCurrentMapNode(), Tinit,
                merged_clouds); // Tnow will be updated to the actual pose of the robot according to
                                // ndt-d2d registration
            graph_map_->LocalToWorldMapFrame(Tinit);
        }
        if (registration_succesfull)
        {
            Tnow = Tinit;
            updated = true;
            UpdateSingleMap(merged_clouds, Tnow);
            pose_last_fuse_ = Tnow;

            if (save_merged_cloud_)
            {
                Eigen::Affine3d sensorPose_inv = sensorPose_.inverse();
                for (size_t i = 0; i < clouds.size(); i++)
                {
                    transformPointCloudInPlace(sensorPose_inv, merged_clouds[i]);
                    pcl::io::savePCDFileASCII("merged_clouds" + ndt_generic::toString(i) + "_" +
                                                  ndt_generic::toString(nb_frames++) + ".pcd",
                                              merged_clouds[i]);
                }
            }
        }
        else
            cout << "REG error" << endl;

        merged_clouds.clear();
        merged_clouds_count = 0;
    }
    fuse_prev = fuse_frame;

    return updated;
}
template <class PointT>
bool GraphMapFuser::ProcessFrameStatic(std::vector<pcl::PointCloud<PointT>>& clouds,
                                       Eigen::Affine3d& Tnow,
                                       Eigen::Affine3d& Tmotion)
{

    static bool fuse_prev = true;
    bool fuse_frame = true;
    static std::vector<pcl::PointCloud<PointT>> merged_clouds;
    static unsigned int merged_clouds_count = 0;
    bool registration_succesfull = false;
    bool updated = false;
    if (!initialized_)
    {
        cerr << "fuser not initialized" << endl;
        return false;
    }

    nr_frames_++;
    Tnow = Tnow * Tmotion;

    fuse_frame = FuseFrame(Tnow, Tmotion); // return true if vehicle standing still
    if (fuse_frame == true && fuse_prev == false)
    { // stannade precis till
        for (size_t i = 0; i < clouds.size(); i++)
        {
            transformPointCloudInPlace(sensorPose_, clouds[i]);
            merged_clouds[i] += clouds[i];
        }
        merged_clouds_count++;
    }
    else if (fuse_frame == true && fuse_prev == true)
    { // been standing still since last frame
        for (size_t i = 0; i < clouds.size(); i++)
        {
            transformPointCloudInPlace(sensorPose_, clouds[i]);
            merged_clouds[i] += clouds[i];
        }
        merged_clouds_count++;
    }
    else if (fuse_frame == false && fuse_prev == true)
    { // started moving, should register towards previous map here
        UpdateSingleMap(merged_clouds, Tnow);
        pose_last_fuse_ = Tnow;
        merged_clouds.clear();
        merged_clouds_count = 0;

        if (graph_map_->Size() >= 2)
        {

            Eigen::Affine3d diff =
                graph_map_->GetPreviousNodePose().inverse() * graph_map_->GetCurrentNodePose();

            double score;
            cout << "register" << endl;
            NDTD2DRegTypePtr ndt_reg = boost::dynamic_pointer_cast<NDTD2DRegType>(registrator_);
            bool registration_succesfull; // register here

            if (registration_succesfull)
            {

                updated = true;
                cout << "REG success" << endl;
                Tnow = graph_map_->GetPreviousNodePose() * diff * sensorPose_.inverse();
                graph_map_->UpdateLink(graph_map_->GetPreviousNode(), graph_map_->GetCurrentNode(),
                                       graph_map_->GetPreviousNodePose().inverse() *
                                           graph_map_->GetCurrentNodePose());
            }
            else
                cout << "REG error" << endl;
        }
    }
    else if (fuse_frame == false && fuse_prev == false)
    { // moving, do nothing
    }

    fuse_prev = fuse_frame;

    return updated;
}

template <class PointT>
void GraphMapFuser::UpdateSingleMap(std::vector<pcl::PointCloud<PointT>>& clouds,
                                    Eigen::Affine3d& Tnow)
{ // specified in the global frame
    // cout<<"Update"<<endl;
    graph_map_->WorldToLocalMapFrame(Tnow);
    for (size_t i = 0; i < clouds.size(); i++)
    {
        transformPointCloudInPlace(Tnow, clouds[i]);
    }
    // Update map, provided transform is the pose of the sensor in
    // the world which is where the scan was taken from
    graph_map_->GetCurrentNode()->updateMap(Tnow * sensorPose_, clouds);

    graph_map_->LocalToWorldMapFrame(Tnow);
    pose_last_fuse_ = Tnow;
}

template <class PointT>
void GraphMapFuser::UpdateMultipleMaps(std::vector<pcl::PointCloud<PointT>>& clouds,
                                       Eigen::Affine3d& Tnow)
{ // specified in the global frame

    std::vector<MapNodePtr> closest_nodes =
        graph_map_->GetClosestNodes(Tnow * sensorPose_, 1.0, true);
    if (closest_nodes.size() == 0)
        cout << "close nodes There should always be a node within interchange radius" << endl;
    else if (closest_nodes.size() == 1)
    {
        UpdateSingleMap(clouds, Tnow);
        cout << "close nodes, one" << endl;
    }
    else
    {
        for (int i = 0; i < closest_nodes.size(); i++)
        {
            std::vector<pcl::PointCloud<PointT>> cloud_map_i(clouds.size());
            for (size_t j = 0; j < clouds.size(); j++)
            {
                copyPointCloud(cloud_map_i[j], clouds[j]);
            }
            MapNodePtr map_i = closest_nodes[i];
            Eigen::Affine3d Tinit = Tnow;
            graph_map_->WorldToLocalMapFrame(Tinit, map_i);
            for (size_t j = 0; j < cloud_map_i.size(); j++)
            {
                transformPointCloudInPlace(Tinit, cloud_map_i[j]);
            }
            map_i->updateMap /*<PointT>*/ (
                Tinit * sensorPose_,
                cloud_map_i); // Update map, provided transform is the pose of the sensor in the
                              // world which is where the scan was taken from
            cout << "update node=" << map_i->GetPose().translation() << endl;
        }
        pose_last_fuse_ = Tnow;
    }
}

} // namespace graph_map
} // namespace perception_oru
