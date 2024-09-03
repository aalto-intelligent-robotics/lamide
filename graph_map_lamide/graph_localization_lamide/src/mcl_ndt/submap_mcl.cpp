#include "graph_localization_lamide/mcl_ndt/submap_mcl.h"

#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"

namespace perception_oru
{
namespace graph_localization
{

SubmapMCLType::SubmapMCLType(LocalisationParamPtr param)
    : LocalizationType(param)
{
    if (SubmapMCLParamPtr mclParam = boost::dynamic_pointer_cast<SubmapMCLParam>(param))
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
                ROS_INFO_STREAM("MCL-NDT is only valid for NDT or NDT-DL maps");
                exit(0);
            }
        }

        counter = 0;
        forceSIR = mclParam->forceSIR;
        motion_model_ = mclParam->motion_model;
        n_particles_ = mclParam->n_particles;
        SIR_varP_threshold = mclParam->SIR_varP_threshold;
        z_filter_min = mclParam->z_filter_min;
        score_cell_weight = mclParam->score_cell_weight;
        initialized_ = false;
        forceSIR = mclParam->forceSIR;
        resolution_local_factor_ = mclParam->resolution_local_factor;
        load_previous_heatmap_ = mclParam->load_previous_heatmap;
        save_heatmap_ = mclParam->save_heatmap;
        heatmap_file_path_ = mclParam->heatmap_file_path;
        uniform_initialization_ = mclParam->uniform_initialization;
        convergance_rate_ = mclParam->convergance_rate;
        time_t t;
        srand(time(&t));
        ros::NodeHandle nh("~");
        part_pub = nh.advertise<geometry_msgs::PoseArray>("pose_particles", 100);

        bool created = false;
        if (load_previous_heatmap_ && HeatMapInterface::LoadHeatMap(heatmap_file_path_, heatmap_))
        {
            if (!heatmap_->SetAndVerifyInterface(graph_map_))
            {
                cerr << "Heatmap is inconsistent with previous map" << endl;
                // heatmap_.reset();
                heatmap_ =
                    boost::shared_ptr<HeatMapInterface>(new HeatMapInterface(graph_map_, 0.5));
                created = true;
            }
            else
                cout << "Heatmap ok" << endl;
        }
        else
        {
            heatmap_ = boost::shared_ptr<HeatMapInterface>(new HeatMapInterface(graph_map_, 0.5));
            created = true;
        }
        if (created && save_heatmap_)
            HeatMapInterface::SaveHeatMap(heatmap_file_path_, heatmap_);

        graph_map_->SwitchToMapNode(*graph_map_->begin());
        Tfirst_ = (*graph_map_->begin())->GetPose();
    }
    else
    {
        std::cerr << "Cannot create MCLNDType. Illegal type for parameter param" << endl;
        exit(0);
    }
    cout << "Initialized submap-mcl type" << endl;
}

NDTMap* SubmapMCLType::GetCurrentNodeNDTMap(MapNodePtr nodePtr)
{

    if (NDTMapPtr p = boost::dynamic_pointer_cast<NDTMapType>(nodePtr->GetMap()))
        return p->GetNDTMap();

    if (NDTDLMapPtr p = boost::dynamic_pointer_cast<NDTDLMapType>(nodePtr->GetMap()))
        return p->GetNDTMapFlat();

    return NULL;
}
bool SubmapMCLType::Reset()
{
    pf.pcloud.clear();
    pf_delayed.pcloud.clear();
    sinceSIR_ = 0;
    pose_last_update_ = Eigen::Affine3d::Identity();
    pose_ = Eigen::Affine3d::Identity();
    prev_pose_ = Eigen::Affine3d::Identity();
    velocity_ = Eigen::Affine3d::Identity();
    counter = 0;
    initialized_ = false;
}

void SubmapMCLType::AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts)
{
    int Nn = 0;
    double t_pseudo = getDoubleTime();
    if (heatmap_ == NULL)
    {
        cerr << "No heatmap to select map" << endl;
    }

#pragma omp parallel num_threads(12)
    {
#pragma omp for
        for (int i = 0; i < pf.size() + pf_delayed.size(); i++)
        {

            MapNodePtr mapPtr = NULL;
            particle& p = i < pf.size() ? pf.pcloud[i] : pf_delayed.pcloud[i - pf.size()];
            Eigen::Vector3d p_pos = (Tfirst_ * p.GetAsAffine()).translation(); // in world frame
            if (heatmap_->GetMapForPoint(p_pos, mapPtr, graph_map_) < -0.1 || mapPtr == NULL)
            {
                p.SetLikelihood(0);
                // cerr<<"Couldnt get map for particle"<<endl;
                continue;
            }

            NDTMap* map = GetCurrentNodeNDTMap(mapPtr);
            Eigen::Affine3d T =
                mapPtr->GetPose().inverse() * Tfirst_ * p.GetAsAffine(); // Change of Frame
            double score = 1;

            if (ndts.size() == 0)
                fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
            Nn = ndts.size();

            for (int n = 0; n < ndts.size(); n++)
            {
                Eigen::Vector3d m = T * ndts[n]->getMean();
                if (m[2] < z_filter_min)
                    continue;

                perception_oru::NDTCell* cell;
                pcl::PointXYZL p;
                p.x = m[0];
                p.y = m[1];
                p.z = m[2];
                // std::cout<<"Get Cell"<<std::endl;
                if (map->getCellAtPoint(p, cell))
                {
                    // if(map.getCellForPoint(p,cell)){
                    if (cell == NULL)
                        continue;
                    if (cell->hasGaussian_)
                    {
                        Eigen::Matrix3d covCombined = cell->getCov() + T.rotation() *
                                                                           ndts[n]->getCov() *
                                                                           T.rotation().transpose();
                        Eigen::Matrix3d icov;
                        bool exists;
                        double det = 0;
                        covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                        if (!exists)
                            continue;
                        double l =
                            (cell->getMean() - m)
                                .dot(icov * (cell->getMean() - m)); //(cell->getMean()-m).norm()
                        if (l * 0 != 0)
                            continue;
                        score +=
                            score_cell_weight + (1.0 - score_cell_weight) * exp(-0.05 * l / 2.0);
                    }
                }
            }
            p.SetLikelihood(score);
        }
    } //#pragma
    t_pseudo = getDoubleTime() - t_pseudo;
}
void SubmapMCLType::Resample()
{
    if (pf.pcloud.size() + pf_delayed.pcloud.size() <= 0)
    {
        cerr << "Particle cloud empty" << endl;
        exit(0);
    }
    if (uniform_initialization_)
    {
        int target_particles =
            n_particles_ + convergance_rate_ * (pf.size() + pf_delayed.size() - n_particles_);
        pf.Merge(pf_delayed, true);
        pf.SIRUpdate(target_particles);
        pf.normalize();
    }
    else
    {
        pf.Merge(pf_delayed, true);
        pf.SIRUpdate(n_particles_);
        pf.normalize();
    }
}
void SubmapMCLType::SIResampling()
{

    if (forceSIR)
    {
        Resample();
    }
    else
    {
        double varP = 0;
        for (int i = 0; i < pf.size(); i++)
        {
            varP += (pf.pcloud[i].GetProbability() - 1.0 / pf.size()) *
                    (pf.pcloud[i].GetProbability() - 1.0 / pf.size());
        }
        varP /= pf.size();
        varP = sqrt(varP);
        // fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf) itr since SIR=
        // %d\n",varP,pf.size(), Nn, t_pred,t_pseudo,sinceSIR_);
        if (varP > /*0.006*/ SIR_varP_threshold || sinceSIR_ > /*25*/ SIR_max_iters_wo_resampling_)
        {
            // fprintf(stderr,"-SIR- ");
            sinceSIR_ = 0;
            Resample();
        }
        else
        {
            sinceSIR_++;
        }
    }
}

void SubmapMCLType::ComputeMotionCovar(const Eigen::Affine3d& Tmotion,
                                       Eigen::Matrix<double, 6, 1>& motion_cov)
{

    /*  Eigen::Matrix<double, 6,6> motion_model_m(motion_model.data());

    Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
    motion_model_.getCovMatrix(Tmotion);
    ndt_generic::normalizeEulerAngles(rot);
    Eigen::Matrix<double,6,1> incr;
    incr<< fabs(tr[0]),fabs(tr[1]),fabs(tr[2]), fabs(rot[0]), fabs(rot[1]), fabs(rot[2]);
    motion_cov = motion_model_m*incr;*/
    motion_cov = motion_model_.GetCovarianceDiagonal(Tmotion);
    motion_cov += motion_model_.params.offset;
}

void SubmapMCLType::OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise)
{

    Eigen::Matrix<double, 6, 1> motion_cov;
    if (!disable_noise)
        ComputeMotionCovar(Tmotion, motion_cov);
    else
        motion_cov << 0, 0, 0, 0, 0, 0;

    if (!enable_localisation_)
        pf.predict(Tmotion, 0, 0, 0, 0, 0,
                   0); // Be aware numerical drift here, cannot be used with ground truth data
    else
        pf.predict(Tmotion, motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3],
                   motion_cov[4], motion_cov[5]);
}

// ██████╗ ██████╗ ███████╗██████╗ ██╗ ██████╗████████╗
// ██╔══██╗██╔══██╗██╔════╝██╔══██╗██║██╔════╝╚══██╔══╝
// ██████╔╝██████╔╝█████╗  ██║  ██║██║██║        ██║
// ██╔═══╝ ██╔══██╗██╔══╝  ██║  ██║██║██║        ██║
// ██║     ██║  ██║███████╗██████╔╝██║╚██████╗   ██║
// ╚═╝     ╚═╝  ╚═╝╚══════╝╚═════╝ ╚═╝ ╚═════╝   ╚═╝

bool SubmapMCLType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                     const Eigen::Affine3d& Tmotion,
                                     const Eigen::Affine3d& Tsensor)
{

    bool updated = false;
    counter++;
    static Eigen::Affine3d Tmot_accum = Tmotion;
    if (counter > 0)
        Tmot_accum = Tmot_accum * Tmotion;
    if (min_keyframe_dist_ < 0.001 && Tmot_accum.translation().norm() < 0.001)
    { // no motion wher
        ROS_INFO_STREAM("No movement");
        return false;
    }
    else if (Tmot_accum.translation().norm() < min_keyframe_dist_)
    {
        ROS_INFO_STREAM("Odom only");
        SetPose(Tfirst_ * pf.getMeanFiltered(percent_inliers) *
                Tmot_accum); // Use only odometry to predict movement
        return false;
    }
    else if (!enable_localisation_)
    {                                         // dont actually score the particles
        OdometryPrediction(Tmot_accum, true); // disable prediction noise if no motion
        ROS_INFO_STREAM("Localization not enabled");
    }
    else
    {
        ROS_INFO_STREAM("Odometry prediction");
        OdometryPrediction(Tmot_accum, false); // particle score and resampling percedure

        // else update particles
        perception_oru::NDTMap local_map(
            new perception_oru::LazyGrid(resolution * resolution_local_factor_));
        local_map.loadPointCloud(cloud);
        local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
        std::vector<perception_oru::NDTCell*> ndts_original = local_map.getAllCellsNoCopy();
        AssignParticleScore(ndts_original);
        SIResampling();
    }

    Tmot_accum = Eigen::Affine3d::Identity();
    SetPose(Tfirst_ * pf.getMeanFiltered(percent_inliers));

    if (visualize_)
    {
        // Eigen::Affine3d Tidentity = Eigen::Affine3d::Identity();
        geometry_msgs::PoseArray particles_msg = ParticlesToMsg(Tfirst_, pf.pcloud);
        part_pub.publish(particles_msg);
    }
    updated = true;
    return updated;
}

bool SubmapMCLType::UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                     const Eigen::Affine3d& Tmotion,
                                     const Eigen::Affine3d& Tsensor)
{

    if (clouds.empty())
        return false;

    return UpdateAndPredict(clouds[0], Tmotion, Tsensor);
}

std::string SubmapMCLType::ToString()
{

    std::stringstream ss;
    ss << LocalizationType::ToString();
    // ss<<graph_map_->ToString();
    ss << "NDTMCL:" << endl;
    ss << "number of particles: " << n_particles_ << endl;
    ss << "resolution: " << resolution << endl;
    ss << "resolution local_factor: " << resolution_local_factor_ << endl;
    ss << "force SIR: " << std::boolalpha << forceSIR << endl;
    ss << "SIR var Probability threshold: " << SIR_varP_threshold << endl;
    ss << "motion model: ";
    ss << motion_model_.params.getDescString() << endl;
    // for (int i = 0; i < motion_model_.params.motion_model.size(); i++) { if (i % 6 == 0) {ss <<
    // endl;} ss<<motion_model_.params.motion_model[i] << " " << endl; } ss<<"motion model offset:
    // "; for (int i = 0; i < motion_model_offset.size(); i++) { if (i % 6 == 0) {ss << endl;};
    // ss<<motion_model_offset[i] << " " << endl; }
    return ss.str();
}
void SubmapMCLType::init(const Eigen::Affine3d& pose,
                         const Vector6d& variance,
                         particle_filter_3d& part,
                         bool keep_current_cloud)
{

    Eigen::Affine3d Tplocal = Tfirst_.inverse() * pose;
    Eigen::Vector3d euler = Tplocal.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(euler);
    Eigen::Vector3d pos = Tplocal.translation();
    part.initializeNormalRandom(ratio_particles_initialize * n_particles_, pos(0), pos(1), pos(2),
                                euler(0), euler(1), euler(2), variance(0), variance(1), variance(2),
                                variance(3), variance(4), variance(5), keep_current_cloud);
}

void SubmapMCLType::InitializeLocalization(const Eigen::Affine3d& pose,
                                           const Vector6d& variance,
                                           bool keep_current_cloud)
{ // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
    pf_delayed.pcloud.clear();
    if (!keep_current_cloud)
        SetPose(pose);
    init(pose, variance, pf, keep_current_cloud);

    initialized_ = true;
    if (visualize_)
    {
        // Eigen::Affine3d Tidentity = Eigen::Affine3d::Identity();
        particle_filter_3d ptmp;
        ptmp.Merge(pf);
        ptmp.Merge(pf_delayed);
        geometry_msgs::PoseArray particles_msg = ParticlesToMsg(Tfirst_, ptmp.pcloud);
        part_pub.publish(particles_msg);
    }
}

void SubmapMCLType::InitializeLocalization(const Eigen::Affine3d& pose, const Vector6d& variance)
{ // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
    InitializeLocalization(pose, variance, false);
}
void SubmapMCLType::DelayedInitialization(const Eigen::Affine3d& pose, const Vector6d& variance)
{ // Samples particles, skips the prediction and then fuses the initialization. This is usefull to
  // efficiently integrate the particles.
    Eigen::Affine3d Tplocal = Tfirst_.inverse() * pose;
    Eigen::Vector3d euler = Tplocal.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(euler);
    Eigen::Vector3d pos = Tplocal.translation();
    double avg_prob = 0;
    pf.MaxProbability(avg_prob);

    pf_delayed.initializeNormalRandom(ratio_particles_initialize * n_particles_, pos(0), pos(1),
                                      pos(2), euler(0), euler(1), euler(2), variance(0),
                                      variance(1), variance(2), variance(3), variance(4),
                                      variance(5), avg_prob);
    if (visualize_)
    {
        particle_filter_3d ptmp;
        ptmp.Merge(pf);
        ptmp.Merge(pf_delayed);
        geometry_msgs::PoseArray particles_msg = ParticlesToMsg(Tfirst_, ptmp.pcloud);
        part_pub.publish(particles_msg);
    }
}
/*void SubmapMCLType::DelayedInitialization(Eigen::MatrixXd &cov,  const Eigen::Matrix<double,6,1>
&scaling,  const Eigen::Affine3d &pose){ //Samples particles, skips the prediction and then fuses
the initialization. This is usefull to efficiently integrate the particles.

  Eigen::Affine3d Tplocal = Tfirst_.inverse()*pose;
  pf_delayed.initializeByCovarianceMatrix(ratio_particles_initialize*n_particles_, Tplocal, cov,
scaling); if(visualize_){ particle_filter_3d ptmp; ptmp.Merge(pf); ptmp.Merge(pf_delayed);
    geometry_msgs::PoseArray particles_msg = ParticlesToMsg( Tfirst_, ptmp.pcloud);
    part_pub.publish(particles_msg);
  }
}*/
void SubmapMCLType::UniformInitialization(const Vector6d& spread, const Vector6d& resolution)
{

    Eigen::MatrixXd d(100, 100);
    const Eigen::Affine3d Tsens_inv = sensor_pose_.inverse();
    const Eigen::Affine3d Tfirst_inv = (*graph_map_->begin())->GetPose().inverse();

    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> position_clouds;
    position_clouds.resize(graph_map_->Size(), NULL);
    for (int i = 0; i < graph_map_->Size(); i++)
    {
        MapNodePtr nodePtr = graph_map_->GetNode(i);
        ndt_generic::Affine3dSTLVek obs_vek = nodePtr->GetObservationVector(false);
        position_clouds[i] =
            pcl::PointCloud<pcl::PointXYZL>::Ptr((new pcl::PointCloud<pcl::PointXYZL>()));
        for (auto& obs : obs_vek)
        {
            Eigen::Vector3d p = (Tfirst_inv * obs * sensor_pose_.inverse()).translation();
            GenerateSpreadPos(p, spread, resolution, position_clouds[i]);
        }
    }

    pcl::VoxelGrid<pcl::PointXYZL> sor;
    pcl::PointCloud<pcl::PointXYZL>::Ptr merged =
        pcl::PointCloud<pcl::PointXYZL>::Ptr((new pcl::PointCloud<pcl::PointXYZL>()));
    int size = 0;
    for (auto cloud : position_clouds)
    {
        (*merged) += (*cloud);
    }
    sor.setInputCloud(merged);
    sor.setLeafSize(resolution(0), resolution(1), resolution(2));
    pcl::PointCloud<pcl::PointXYZL>::Ptr position_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZL>());
    sor.filter(*position_cloud_filtered);
    ndt_generic::Affine3dSTLVek Particles;
    GenerateSpreadRot(position_cloud_filtered, spread, resolution, Particles);
    pf_delayed.initializeFromVector(Particles);
    SetPose(Eigen::Affine3d::Identity());

    initialized_ = true;
    if (visualize_)
    {
        // Eigen::Affine3d Tidentity = Eigen::Affine3d::Identity();
        geometry_msgs::PoseArray particles_msg = ParticlesToMsg(Tfirst_, pf_delayed.pcloud);
        part_pub.publish(particles_msg);
    }
}
void SubmapMCLType::GenerateSpreadRot(pcl::PointCloud<pcl::PointXYZL>::Ptr& points,
                                      const Vector6d& spread,
                                      const Vector6d& resolution,
                                      ndt_generic::Affine3dSTLVek& Poses)
{

    // Poses.push_back(ndt_generic::xyzrpyToAffine3d(p.x, p.y, p.z, i, j, k));
    Vector6d res = resolution;
    Vector6d adjust;
    adjust << 0.000000001, 0.000000001, 0.000000001, 0.000000001, 0.000000001, 0.000000001;
    res = res - adjust;
    int m_dim = 0, n_dim = 0, k_dim = 0;

    for (double i = -spread(3); i <= spread(3); i += res(3))
        m_dim++;

    for (double i = -spread(4); i <= spread(4); i += res(4))
        n_dim++;

    for (double i = -spread(5); i <= spread(5); i += res(5))
        k_dim++;

    std::vector<std::tuple<double, double, double>> angles;

    int size_angles = m_dim * n_dim * k_dim;
    angles.resize(size_angles);
    int count_angle_repetitions = 0;

    for (double i = -spread(3); i <= spread(3); i += res(3))
    {
        for (double j = -spread(4); j <= spread(4); j += res(4))
        {
            for (double k = -spread(5); k <= spread(5); k += res(5))
            {
                angles[count_angle_repetitions++] = std::make_tuple(i, j, k);
            }
        }
    }
    Poses.resize(angles.size() * points->size());

#pragma omp parallel num_threads(12)
    {
#pragma omp for
        for (int x = 0; x < points->size() * angles.size(); x++)
        {
            int idx = x / size_angles;
            Poses[x] = Eigen::Translation<double, 3>((*points)[idx].x, (*points)[idx].y,
                                                     (*points)[idx].z) *
                       Eigen::AngleAxis<double>(std::get<0>(angles[x % size_angles]),
                                                Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxis<double>(std::get<1>(angles[x % size_angles]),
                                                Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxis<double>(std::get<2>(angles[x % size_angles]),
                                                Eigen::Vector3d::UnitZ());
        }
    }
}
void SubmapMCLType::GenerateSpreadPos(const Eigen::Vector3d& position,
                                      const Vector6d& spread,
                                      const Vector6d& resolution,
                                      pcl::PointCloud<pcl::PointXYZL>::Ptr cloud)
{
    if (cloud == NULL)
        std::cerr << "Error cloud NULL" << std::endl;

    for (float i = position(0) - spread(0); i <= position(0) + spread(0) + 0.000001;
         i += resolution(0))
    {
        for (float j = position(1) - spread(1); j <= position(1) + spread(1) + 0.000001;
             j += resolution(1))
        {
            for (float k = position(2) - spread(2); k <= position(2) + spread(2) + 0.000001;
                 k += resolution(2))
            {
                pcl::PointXYZL p;
                p.x = i;
                p.y = j;
                p.z = k;
                cloud->push_back(p);
            }
        }
    }
}
SubmapMCLParam::SubmapMCLParam()
{
}

std::string SubmapMCLParam::ToString()
{
    stringstream ss;
    ss << LocalisationParam::ToString();
    ss << "MCL-NDT parameters:" << endl;
    ss << "z_filter_min=" << z_filter_min << endl;
    ss << "n_particles=" << n_particles << endl;
    ss << "SIR_max_iters_wo_resampling=" << SIR_max_iters_wo_resampling << endl;
    ss << "forceSIR=" << std::boolalpha << forceSIR << endl;
    return ss.str();
}

void SubmapMCLParam::GetParamFromRos()
{
    LocalisationParam::GetParamFromRos();
    cout << "Submap MCL localisation parameters from ROS" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("z_filter_height", z_filter_min, -10000.0);
    nh.param("particle_count", n_particles, 250);
    nh.param("resolution_local_factor", resolution_local_factor, 1.0);
    nh.param("SIR_max_iters_wo_resampling", SIR_max_iters_wo_resampling, 30);
    nh.param("force_SIR", forceSIR, false);
    cout << "Fetched localisation parameters from ros" << endl;
    // cout<<ToString()<<endl;
}

} // namespace graph_localization
} // namespace perception_oru
