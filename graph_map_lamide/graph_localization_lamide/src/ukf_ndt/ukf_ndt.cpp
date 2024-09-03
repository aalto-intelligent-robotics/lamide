#include "graph_localization_lamide/ukf_ndt/ukf_ndt.h"

namespace perception_oru
{
namespace graph_localization
{

UKFNDTType::UKFNDTType(LocalisationParamPtr param)
    : LocalizationType(param)
{
    if (UKFNDTParamPtr ukfParam = boost::dynamic_pointer_cast<UKFNDTParam>(param))
    {
        NDTMapPtr current_map =
            boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
        if (current_map != NULL)
        {
            resolution = current_map->GetResolution();
        }
        else
        {

            NDTDLMapPtr current_dl_map =
                boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
            if (current_dl_map != NULL)
            {
                resolution = current_dl_map->GetResolutionFlat();
            }
            else
            {
                ROS_INFO_STREAM("UKF-NDT is only valid for NDT or NDT-DL maps");
                exit(0);
            }
        }

        counter = 0;

        UKF3D::Params params;
        params.range_var = ukfParam->range_var;
        params.min_pos_var = ukfParam->min_pos_var;
        params.min_rot_var = ukfParam->min_rot_var;
        params.range_filter_max_dist = ukfParam->range_filter_max_dist;
        params.nb_ranges_in_update = ukfParam->nb_ranges_in_update;
        params.nb_updates = ukfParam->nb_updates;
        ukf.setParamsUKF(params);

        motion_model_ = ukfParam->motion_model;

        time_t t;
        srand(time(&t));
        ros::NodeHandle nh("~");
        sigma_pub = nh.advertise<geometry_msgs::PoseArray>("pose_sigma", 100);
    }
    else
    {
        std::cerr << "Cannot create UKFNDType. Illegal type for parameter param" << endl;
        exit(0);
    }
}

NDTMap* UKFNDTType::GetCurrentNodeNDTMap()
{
    // Currently exist two different maps of NDT, NDT and NDTDL, find out which we have in the graph
    // and return it...
    {
        NDTMapPtr p =
            boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
        if (p != NULL)
        {
            return p->GetNDTMap();
        }
    }
    {
        NDTDLMapPtr p =
            boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
        return p->GetNDTMapFlat();
    }
    return NULL;
}

void UKFNDTType::ComputeMotionCovar(const Eigen::Affine3d& Tmotion, Eigen::MatrixXd& motion_cov)
{
    motion_cov = motion_model_.getCovMatrix(Tmotion);
    std::cout << "motion_cov :  \n" << motion_cov << std::endl;
    Eigen::Matrix<double, 6, 6> offset = Eigen::MatrixXd::Zero(6, 6);

    for (int i = 0; i < motion_model_.params.offset.rows(); i++)
        offset(i, i) = motion_model_.params.offset(i);

    motion_cov +=
        offset; // commented by henrik perhaps since the offset was not 6x6, added again by daniel
}

void UKFNDTType::OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise)
{

    Eigen::MatrixXd motion_cov;
    ComputeMotionCovar(Tmotion, motion_cov);

    if (disable_noise)
    {
        motion_cov.setZero();
    }

    // Get mean estimate in world frame and find closest node
    //FIXME: max distance
    bool new_map_node = graph_map_->SwitchToClosestMapNode(ukf.getMean(), 107);

    ukf.predict(Tmotion, motion_cov);

    if (visualize_)
    {
        geometry_msgs::PoseArray sigma_msg = SigmasToMsg(ukf.getSigmasAsAffine3d());
        sigma_pub.publish(sigma_msg);
    }
}

bool UKFNDTType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor)
{

    bool updated = false;
    counter++;

    Eigen::Affine3d Tlast = ukf.getMean();
    bool disable_prediciton_noise = false;

    if ((Tlast.translation() - pose_last_update_.translation()).norm() < 0.01 &&
        (pose_last_update_.inverse() * Tlast).rotation().eulerAngles(0, 1, 2).norm() < 0.005)
        disable_prediciton_noise = true;

    OdometryPrediction(Tmotion, disable_prediciton_noise);
    Eigen::Affine3d Tinit = ukf.getMean();

    if (!enable_localisation_ || disable_prediciton_noise)
    {
        SetPose(Tinit); // Use only odometry to predict movement
        return updated;
    }

    if (voxel_filter_size > 0)
    {
        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZL>());
        *cloud2 = cloud;
        pcl::VoxelGrid<pcl::PointXYZL> sor;
        sor.setInputCloud(cloud2);
        sor.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        pcl::PointCloud<pcl::PointXYZL> cloud_filtered;
        sor.filter(cloud);
    }

    ukf.update(cloud, Tsensor);

    Tinit = ukf.getMean();
    pose_last_update_ = Tinit;
    SetPose(Tinit);
    if (visualize_)
    {
        // Visualize the predicted / filtered cloud.
        GraphPlot::PlotScan(ukf.getFilterRaw(), std::string("/fuser"), std::string("ukf_raw"));
        GraphPlot::PlotScan(ukf.getFilterPred(), std::string("/fuser"), std::string("ukf_pred"));

        ros::spinOnce();
    }

    updated = true;
    return updated;
}

bool UKFNDTType::UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor)
{
    std::cout << "using segmented cloud" << std::endl;
    if (clouds.empty())
        return false;

    return UpdateAndPredict(clouds[0], Tmotion, Tsensor);
}

std::string UKFNDTType::ToString()
{

    std::stringstream ss;
    ss << LocalizationType::ToString();
    ss << graph_map_->ToString();
    ss << "NDTUKF:" << endl;
    ss << ukf.getParamsUKF() << std::endl;
    ss << motion_model_.params.getDescString() << endl;
    return ss.str();
}

void UKFNDTType::InitializeLocalization(const Eigen::Affine3d& pose, const Vector6d& variance)
{ // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame

    SetPose(pose);
    pose_last_update_ = GetPose();
    // FIXME: max distance
    bool new_map_node = graph_map_->SwitchToClosestMapNode(pose, 108);
    Eigen::Affine3d Tprev_to_new;
    Tprev_to_new = graph_map_->GetCurrentNodePose().inverse();
    ukf.setMapOffset(Tprev_to_new);
    map_ = this->GetCurrentNodeNDTMap();
    ukf.setMap(map_);
    Eigen::Affine3d pose_local = graph_map_->GetCurrentNodePose().inverse() * pose;
    Eigen::Vector3d pos = pose_local.translation();
    Eigen::Vector3d euler = pose_local.rotation().eulerAngles(0, 1, 2);
    ukf.initializeFilter(pose, variance(0), variance(1), variance(2), variance(3), variance(4),
                         variance(5));
    initialized_ = true;
}

UKFNDTParam::UKFNDTParam()
{
}

std::string UKFNDTParam::ToString()
{
    stringstream ss;
    ss << LocalisationParam::ToString();
    ss << "range_var=" << range_var;
    ss << "min_pos_var=" << min_pos_var;
    ss << "min_rot_var=" << min_rot_var;
    ss << "range_filter_max_dist=" << range_filter_max_dist;
    ss << "nb_ranges_in_update=" << nb_ranges_in_update;
    ss << "nb_updates=" << nb_updates;
    return ss.str();
}

void UKFNDTParam::GetParamFromRos()
{
    LocalisationParam::GetParamFromRos();
    cout << "UKFNDT localisation parameters from ROS" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("range_var", range_var, 1.0);
    nh.param("min_pos_var", min_pos_var, 0.01);
    nh.param("min_rot_var", min_rot_var, 0.01);
    nh.param("range_filter_max_dist", range_filter_max_dist, 1.);
    nh.param("nb_ranges_in_update", nb_ranges_in_update, 100);
    nh.param("nb_updates", nb_updates, 1000);

    cout << "Fetched localisation parameters from ros" << endl;
}

} // namespace graph_localization
} // namespace perception_oru
