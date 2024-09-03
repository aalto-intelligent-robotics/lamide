#include "graph_localization_lamide/mcl_ndt/mcl_ndtdl.h"

#include "graph_localization_lamide/mcl_ndt/mcl_ndt_utils.h"
#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/ndt_dl/point_curv3.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "ndt_generic_lamide/utils.h"
#include "ndt_rviz_lamide/ndt_rviz.h"

namespace perception_oru
{
namespace graph_localization
{

MCLNDTDLType::MCLNDTDLType(LocalisationParamPtr param)
    : LocalizationType(param)
{
    if (MCLNDTDLParamPtr mclParam = boost::dynamic_pointer_cast<MCLNDTDLParam>(param))
    {
        NDTDLMapPtr current_dl_map =
            boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
        if (current_dl_map != NULL)
        {
            resolution = current_dl_map->GetResolutionFlat();
        }
        else
        {
            ROS_INFO_STREAM("MCL-NDTDL is only valid for NDT-DL maps");
            exit(0);
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
        resolution_sensor = resolution;

        time_t t;
        srand(time(&t));
        ros::NodeHandle nh("~");
        part_pub = nh.advertise<geometry_msgs::PoseArray>("pose_particles", 100);
    }
    else
    {
        std::cerr << "Cannot create MCLNDDLType. Illegal type for parameter param" << endl;
        exit(0);
    }
}

std::vector<NDTMap*> MCLNDTDLType::GetCurrentNodeNDTMaps()
{
    {
        NDTDLMapPtr p =
            boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
        return p->GetNDTMaps();
    }
    std::vector<NDTMap*> ret;
    return ret;
}

void MCLNDTDLType::AssignParticleScore(std::vector<std::vector<perception_oru::NDTCell*>> ndts_vec)
{
    int Nn = 0;
    double t_pseudo = getDoubleTime();
    if (ndts_vec.size() > maps_.size())
    {
        std::cout << "Map size missmatch(!)" << std::endl;
        return;
    }
#pragma omp parallel num_threads(12)
    {
#pragma omp for
        for (int i = 0; i < pf.size(); i++)
        {
            double score = 1;

            for (int j = 0; j < ndts_vec.size(); j++)
            {
                std::vector<perception_oru::NDTCell*>& ndts = ndts_vec[j];
                NDTMap* map = maps_[j];

                Eigen::Affine3d T = pf.pcloud[i].GetAsAffine();

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

                    if (map->getCellAtPoint(p, cell))
                    {
                        // if(map.getCellForPoint(p,cell)){
                        if (cell == NULL)
                            continue;
                        if (cell->hasGaussian_)
                        {
                            Eigen::Matrix3d covCombined =
                                cell->getCov() +
                                T.rotation() * ndts[n]->getCov() * T.rotation().transpose();
                            Eigen::Matrix3d icov;
                            bool exists;
                            double det = 0;
                            covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                            if (!exists)
                                continue;
                            double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
                            if (l * 0 != 0)
                                continue;
                            score += score_cell_weight +
                                     (1.0 - score_cell_weight) * exp(-0.1 * l / 2.0); // 0.05
                        }
                        else
                        {
                        }
                    }
                }
            }
            pf.pcloud[i].SetLikelihood(score);

        } ///#pragma
    }
    t_pseudo = getDoubleTime() - t_pseudo;

    pf.normalize();
}

void MCLNDTDLType::SIResampling()
{
    if (forceSIR)
    {
        pf.SIRUpdate();
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
            pf.SIRUpdate();
        }
        else
        {
            sinceSIR_++;
        }
    }
}

void MCLNDTDLType::ComputeMotionCovar(const Eigen::Affine3d& Tmotion,
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

void MCLNDTDLType::OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise)
{

    Eigen::Matrix<double, 6, 1> motion_cov;
    if (!disable_noise)
        ComputeMotionCovar(Tmotion, motion_cov);
    else
        motion_cov << 0, 0, 0, 0, 0, 0;
    // Get mean estimate in world frame and find closest node
    //FIXME: max distance
    bool new_map_node =
        graph_map_->SwitchToClosestMapNode(graph_map_->GetCurrentNodePose() * pf.getMean(), 104);
    if (new_map_node)
    {
        Eigen::Affine3d Tprev_to_new;
        Tprev_to_new =
            (graph_map_->GetPreviousNodePose().inverse() * graph_map_->GetCurrentNodePose())
                .inverse();
        cout << "change frame, transf=" << Tprev_to_new.translation().transpose() << endl;
        pf.predict(Tmotion, motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3],
                   motion_cov[4], motion_cov[5], Tprev_to_new);
        maps_ = this->GetCurrentNodeNDTMaps();
    }
    else
        pf.predict(Tmotion, motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3],
                   motion_cov[4], motion_cov[5]);
}

bool MCLNDTDLType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                    const Eigen::Affine3d& Tmotion,
                                    const Eigen::Affine3d& Tsensor)
{

    // This will create only a single layer map to be matched.

    bool updated = false;
    counter++;

    Eigen::Affine3d Tlast = graph_map_->GetCurrentNodePose() * pf.getMean();
    bool disable_prediciton_noise = false; // if there os
    if ((Tlast.translation() - pose_last_update_.translation()).norm() < 0.01 &&
        (pose_last_update_.inverse() * Tlast).rotation().eulerAngles(0, 1, 2).norm() < 0.005)
        disable_prediciton_noise = true;

    OdometryPrediction(Tmotion, disable_prediciton_noise);
    Eigen::Affine3d Tinit = graph_map_->GetCurrentNodePose() * pf.getMean();

    if (!enable_localisation_ || disable_prediciton_noise)
    {
        SetPose(Tinit); // Use only odometry to predict movement
        return updated;
    }

    perception_oru::NDTMap local_map(new perception_oru::LazyGrid(resolution_sensor));
    local_map.loadPointCloud(cloud);
    local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    std::vector<perception_oru::NDTCell*> ndts_original = local_map.getAllCells();
    std::vector<std::vector<perception_oru::NDTCell*>> ndts_vec_subsampled(1);

    Subsample(ndts_original, ndts_vec_subsampled[0], subsample_level_);
    AssignParticleScore(ndts_vec_subsampled);
    SIResampling();

    Tinit = graph_map_->GetCurrentNodePose() * pf.getMean();
    pose_last_update_ = Tinit;
    SetPose(Tinit);
    if (visualize_)
    {
        geometry_msgs::PoseArray particles_msg =
            ParticlesToMsg(graph_map_->GetCurrentNodePose(), pf.pcloud);
        part_pub.publish(particles_msg);
    }
    if (visualize_sensor_maps_)
    {
        if (ndts_vec_subsampled.size() > 0)
            GraphPlot::PlotNDT(ndts_vec_subsampled[0], "fuser", "mcl_ndtdl0");
    }

    clearNDTCellVec(ndts_original);

    updated = true;
    return updated;
}

bool MCLNDTDLType::UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                    const Eigen::Affine3d& Tmotion,
                                    const Eigen::Affine3d& Tsensor)
{

    bool updated = false;
    counter++;

    Eigen::Affine3d Tlast = graph_map_->GetCurrentNodePose() * pf.getMean();
    bool disable_prediciton_noise = false; // if there os
    if ((Tlast.translation() - pose_last_update_.translation()).norm() < 0.01 &&
        (pose_last_update_.inverse() * Tlast).rotation().eulerAngles(0, 1, 2).norm() < 0.005)
        disable_prediciton_noise = true;

    OdometryPrediction(Tmotion, disable_prediciton_noise);
    Eigen::Affine3d Tinit = graph_map_->GetCurrentNodePose() * pf.getMean();

    if (!enable_localisation_ || disable_prediciton_noise)
    {
        SetPose(Tinit); // Use only odometry to predict movement
        return updated;
    }

    std::vector<std::vector<perception_oru::NDTCell*>> ndts_original(
        clouds.size()); // This holds the memory...
    std::vector<std::vector<perception_oru::NDTCell*>> ndts_vec_subsampled(
        clouds.size()); // This doesn't

    for (size_t i = 0; i < clouds.size(); i++)
    {
        NDTMap local_map(new perception_oru::LazyGrid(resolution_sensor));
        local_map.loadPointCloud(clouds[i]);
        local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
        ndts_original[i] = local_map.getAllCells();
        std::cout << "ndts_orignal[i].size() : " << ndts_original[i].size();
        Subsample(ndts_original[i], ndts_vec_subsampled[i], subsample_level_);
        std::cout << "ndts_vec_subsampled[i] : " << ndts_vec_subsampled[i].size() << std::endl;
    }
    AssignParticleScore(ndts_vec_subsampled);
    SIResampling();

    Tinit = graph_map_->GetCurrentNodePose() * pf.getMean();

    pose_last_update_ = Tinit;
    SetPose(Tinit);
    if (visualize_)
    {
        geometry_msgs::PoseArray particles_msg =
            ParticlesToMsg(graph_map_->GetCurrentNodePose(), pf.pcloud);
        part_pub.publish(particles_msg);
    }
    if (visualize_sensor_maps_)
    {
        for (size_t i = 0; i < ndts_vec_subsampled.size(); i++)
        {
            GraphPlot::PlotNDT(ndts_vec_subsampled[i], "fuser",
                               "mcl_ndtdl" + ndt_generic::toString(i));
        }
    }

    clearVectorOfNDTCellVec(ndts_original);

    updated = true;
    return updated;
}

std::string MCLNDTDLType::ToString()
{

    std::stringstream ss;
    ss << LocalizationType::ToString();
    ss << graph_map_->ToString();
    ss << "NDTDLMCL:" << endl;
    ss << "number of particles: " << n_particles_ << endl;
    ss << "resolution: " << resolution << endl;
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

void MCLNDTDLType::InitializeLocalization(const Eigen::Affine3d& pose, const Vector6d& variance)
{ // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
    SetPose(pose);
    pose_last_update_ = GetPose();
    // FIXME: max distance
    bool new_map_node = graph_map_->SwitchToClosestMapNode(pose, 105);
    maps_ = this->GetCurrentNodeNDTMaps();
    Eigen::Affine3d pose_local = graph_map_->GetCurrentNodePose().inverse() * pose;
    Eigen::Vector3d pos = pose_local.translation();
    Eigen::Vector3d euler = pose_local.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(euler);

    pf.initializeNormalRandom(n_particles_, pos(0), pos(1), pos(2), euler(0), euler(1), euler(2),
                              variance(0), variance(1), variance(2), variance(3), variance(4),
                              variance(5));
    initialized_ = true;
}

MCLNDTDLParam::MCLNDTDLParam()
{
}

std::string MCLNDTDLParam::ToString()
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

void MCLNDTDLParam::GetParamFromRos()
{
    LocalisationParam::GetParamFromRos();
    cout << "MCLNDT localisation parameters from ROS" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("z_filter_height", z_filter_min, -10000.0);
    nh.param("particle_count", n_particles, 250);
    nh.param("SIR_max_iters_wo_resampling", SIR_max_iters_wo_resampling, 30);
    nh.param("force_SIR", forceSIR, false);
    cout << "Fetched localisation parameters from ros" << endl;
    // cout<<ToString()<<endl;
}

} // namespace graph_localization
} // namespace perception_oru
