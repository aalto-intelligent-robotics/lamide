#include "graph_localization_lamide/ukf_ndt/ukf_reg.h"

namespace perception_oru
{
namespace graph_localization
{

UKFRegType::UKFRegType(LocalisationParamPtr param)
    : LocalizationType(param)
{

    if (UKFRegParamPtr ukfParam = boost::dynamic_pointer_cast<UKFRegParam>(param))
    {
        NDTMapPtr current_map =
            boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
        if (current_map != NULL)
        {
        }
        else
        {

            NDTDLMapPtr current_dl_map =
                boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
            if (current_dl_map != NULL)
            {
            }
            else
            {
                ROS_INFO_STREAM("UKF-NDT is only valid for NDT or NDT-DL maps");
                exit(0);
            }
        }
        if (NDTDLRegParamPtr ndtdl_reg_par_ptr =
                boost::dynamic_pointer_cast<NDTDLRegParam>(ukfParam->registration_parameters))
        {
            NDTDLMapPtr dl_map_ptr =
                boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
            ndtdl_reg_par_ptr->resolutions = dl_map_ptr->GetResolutions();
        }
        else if (NDTD2DRegParamPtr ndtd2d_reg_par_ptr =
                     boost::dynamic_pointer_cast<NDTD2DRegParam>(ukfParam->registration_parameters))
            ndtd2d_reg_par_ptr->resolution =
                boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode())
                    ->GetResolution();

        counter = 0;

        UKF3D::Params params;
        params.min_pos_var = ukfParam->min_pos_var;
        params.min_rot_var = ukfParam->min_rot_var;
        ukf.setParamsUKF(params);

        min_obs_variance_ = ukfParam->min_obs_variance;
        force_use_registration_ = ukfParam->force_use_registration;

        // Registration
        RegParamPtr reg_par_ptr_ = ukfParam->registration_parameters;
        regptr_ = GraphFactory::CreateRegistrationType(reg_par_ptr_);
        motion_model_ = ukfParam->motion_model;

        time_t t;
        srand(time(&t));
        ros::NodeHandle nh("~");
        sigma_pub = nh.advertise<geometry_msgs::PoseArray>("pose_sigma", 100);
    }
    else
    {
        std::cerr << "Cannot create UKFRegype. Illegal type for parameter param" << endl;
        exit(0);
    }
}

NDTMap* UKFRegType::GetCurrentNodeNDTMap()
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

void UKFRegType::UpdateMinObservationCov(Eigen::MatrixXd& cov)
{
    if (min_obs_variance_.size() != 6)
    {
        std::cout << "min obs variance not set!" << std::endl;
        exit(0);
    }
    for (int i = 0; i < 6; i++)
    {
        cov(i, i) = this->min_obs_variance_[i];
    }
}

void UKFRegType::ComputeMotionCovar(const Eigen::Affine3d& Tmotion, Eigen::MatrixXd& motion_cov)
{
    motion_cov = motion_model_.getCovMatrix(Tmotion);
    // motion_cov+=motion_model_.params.offset;
}

void UKFRegType::OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise)
{

    Eigen::MatrixXd motion_cov;
    ComputeMotionCovar(Tmotion, motion_cov);

    if (disable_noise)
    {
        motion_cov.setZero();
    }

    // Get mean estimate in world frame and find closest node
    //FIXME: max distance
    bool new_map_node = graph_map_->SwitchToClosestMapNode(ukf.getMean(), 109);

    if (new_map_node)
    { // Need to update the sigma points in order to fit the new map
        Eigen::Affine3d Tprev_to_new;
        Tprev_to_new = graph_map_->GetCurrentNodePose().inverse();
        ukf.setMapOffset(Tprev_to_new);
        map_ = this->GetCurrentNodeNDTMap();
        ukf.setMap(map_);
    }

    ukf.predict(Tmotion, motion_cov);

    if (visualize_)
    {
        geometry_msgs::PoseArray sigma_msg = SigmasToMsg(ukf.getSigmasAsAffine3d());
        sigma_pub.publish(sigma_msg);
    }
}

bool UKFRegType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor)
{

    bool updated = false;
    counter++;

    Eigen::Affine3d Tlast = ukf.getMean();
    bool disable_prediciton_noise = false;

    if ((Tlast.translation() - pose_last_update_.translation()).norm() < 0.03 &&
        (pose_last_update_.inverse() * Tlast).rotation().eulerAngles(0, 1, 2).norm() < 0.01)
        disable_prediciton_noise = true;

    OdometryPrediction(Tmotion, disable_prediciton_noise);
    Eigen::Affine3d Tinit = ukf.getMean();
    if (!enable_localisation_ || disable_prediciton_noise)
    {
        SetPose(Tinit); // Use only odometry to predict movement
        return updated;
    }

    graph_map_->WorldToLocalMapFrame(Tinit);
    bool successful = regptr_->RegisterScan(graph_map_->GetCurrentMapNode(), Tinit, cloud);
    graph_map_->LocalToWorldMapFrame(Tinit);

    if (successful)
    {
        pose_last_update_ = Tinit;

        Eigen::MatrixXd cov(6, 6); // Get the covariance from the registration.
        cov.setZero();
        UpdateMinObservationCov(cov);
        ukf.update(Tinit, cov);
    }
    updated = successful;
    SetPose(Tinit);

    return updated;
}

bool UKFRegType::UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor)
{

    if (clouds.empty())
        return false;

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

    graph_map_->WorldToLocalMapFrame(Tinit);
    bool successful = regptr_->RegisterScan(graph_map_->GetCurrentMapNode(), Tinit, clouds);
    graph_map_->LocalToWorldMapFrame(Tinit);
    if (successful | force_use_registration_)
    {
        pose_last_update_ = Tinit;

        Eigen::MatrixXd cov(6, 6); // Get the covariance from the registration.
        cov.setZero();
        UpdateMinObservationCov(cov);
        ukf.update(Tinit, cov);
        Tinit = ukf.getMean();
    }
    updated = successful;
    SetPose(Tinit);
    return updated;
}

std::string UKFRegType::ToString()
{

    std::stringstream ss;
    ss << LocalizationType::ToString();

    ss << "NDTUKF:" << endl;
    ss << ukf.getParamsUKF() << std::endl;
    ss << motion_model_.params.getDescString() << endl;
    ss << "min observation var:" << endl;
    for (int i = 0; i < min_obs_variance_.size(); i++)
    {
        ss << "|" << min_obs_variance_[i];
    }
    ss << "|" << endl;
    ss << regptr_->ToString() << endl;

    return ss.str();
}

void UKFRegType::InitializeLocalization(const Eigen::Affine3d& pose, const Vector6d& variance)
{ // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame

    SetPose(pose);
    pose_last_update_ = GetPose();
    // FIXME: max distance
    bool new_map_node = graph_map_->SwitchToClosestMapNode(pose, 110);
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

UKFRegParam::UKFRegParam()
{
    min_obs_variance.resize(6);
    min_obs_variance[0] = 0.2;
    min_obs_variance[1] = 0.2;
    min_obs_variance[2] = 0.1;
    min_obs_variance[3] = 0.05;
    min_obs_variance[4] = 0.05;
    min_obs_variance[5] = 0.05;
}

std::string UKFRegParam::ToString()
{

    std::stringstream ss;
    ss << LocalisationParam::ToString();

    ss << "NDTUKF:" << endl;
    ss << motion_model.params.getDescString() << endl;
    ss << "min observation var:" << endl;
    for (int i = 0; i < min_obs_variance.size(); i++)
    {
        ss << "|" << min_obs_variance[i];
    }
    ss << "|" << endl;
    // ss << registration_parameters->ToString() << endl;

    return ss.str();
}

void UKFRegParam::GetParamFromRos()
{
    LocalisationParam::GetParamFromRos();
    cout << "UKFReg localisation parameters from ROS" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("min_pos_var", min_pos_var, 0.01);
    nh.param("min_rot_var", min_rot_var, 0.01);
    std::string dataset = "";
    nh.param<std::string>("dataset", dataset, "");
    nh.param("force_use_registration", force_use_registration, true);

    min_obs_variance.resize(6);
    for (int i = 0; i < min_obs_variance.size(); i++)
        nh.param("min_obs_variance_" + to_string(i + 1), min_obs_variance[i], 0.1);

    if (registration_parameters != NULL)
    {
        registration_parameters->GetParametersFromRos();
    }

    cout << "Successfully retreived ros parameters" << endl;
}

} // namespace graph_localization
} // namespace perception_oru
