#include "graph_localization_lamide/mcl_ndt/mcl_ndt.h"

#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"

#include "graph_map_lamide/utils.h"
#include <limits>

namespace perception_oru
{
namespace graph_localization
{

MCLNDTType::MCLNDTType(LocalisationParamPtr param)
    : LocalizationType(param)
{
    if (MCLNDTParamPtr mclParam = boost::dynamic_pointer_cast<MCLNDTParam>(param))
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
        SIRtype_ = mclParam->SIRtype;
        SIR_varP_threshold = mclParam->SIR_varP_threshold;
        forceSIR = mclParam->forceSIR;
        SIR_max_iters_wo_resampling_ = mclParam->SIR_max_iters_wo_resampling;
        motion_model_ = mclParam->motion_model;
        n_particles_ = mclParam->n_particles;
        z_filter_min = mclParam->z_filter_min;
        score_cell_weight = mclParam->score_cell_weight;
        initialized_ = false;
        resolution_local_factor_ = mclParam->resolution_local_factor;
        subsample_level_ = mclParam->subsample_level;
        enable_localisation_ = mclParam->enable_localisation;
        max_map_distance_ = mclParam->max_map_distance_;
        ratio_particles_initialize = mclParam->ratio_particles_initialize;
        min_ratio_eff_particles_ = mclParam->min_ratio_eff_particles;
        map_change_hysteresis_ = mclParam->map_change_hysteresis;
        lifelong_ = mclParam->lifelong;
        lifelong_pf_size_th_ = mclParam->lifelong_pf_size_th;
        lifelong_max_numpoints_ = mclParam->lifelong_max_numpoints;
        lifelong_max_occupancy_ = mclParam->lifelong_max_occupancy;
        lifelong_max_z_ = mclParam->lifelong_max_z;
        lifelong_sensor_noise_ = mclParam->lifelong_sensor_noise;
        local_colors_ = mclParam->local_colors;
        cluster_update_ = mclParam->cluster_update;

        time_t t;
        srand(time(&t));
        ros::NodeHandle nh("~");
        part_pub = nh.advertise<geometry_msgs::PoseArray>("pose_particles", 100);
    }
    else
    {
        std::cerr << "Cannot create MCLNDType. Illegal type for parameter param" << endl;
        exit(0);
    }
}

NDTMap* MCLNDTType::GetCurrentNodeNDTMap()
{
    // Currently exist two different maps of NDT, NDT and NDTDL, find out which we have in the graph
    // and return it...
    {
        NDTMapPtr p = boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentMapNode());
        if (p != NULL)
        {
            return p->GetNDTMap();
        }
    }
    {
        NDTDLMapPtr p = boost::dynamic_pointer_cast<NDTDLMapType>(graph_map_->GetCurrentMapNode());
        return p->GetNDTMapFlat();
    }
    return NULL;
}

// ███████╗ ██████╗ ██████╗ ██████╗ ███████╗
// ██╔════╝██╔════╝██╔═══██╗██╔══██╗██╔════╝
// ███████╗██║     ██║   ██║██████╔╝█████╗
// ╚════██║██║     ██║   ██║██╔══██╗██╔══╝
// ███████║╚██████╗╚██████╔╝██║  ██║███████╗
// ╚══════╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝

void MCLNDTType::AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts)
{
    // debug
    // int totalHits = 0;
    // int maxHits = 0;
    //
    int Nn = 0;
    std::vector<perception_oru::NDTCell*> ndts_filtered;
    double t_pseudo = getDoubleTime();
    srand(time(NULL));

    for (int i = 0; i < ndts.size(); i++)
    {
        double p = ((double)rand()) / RAND_MAX;
        if (ndts[i] != NULL && p < subsample_level_ && ndts[i]->getMean()(2) > z_filter_min)
            ndts_filtered.push_back(ndts[i]);
    }
    if (ndts_filtered.size() == 0)
    {
        std::cerr << "no usable cells in scan" << endl;
        return;
    }

#pragma omp parallel num_threads(12)
    {
#pragma omp for
        for (int i = 0; i < pf.size(); i++)
        {
            unsigned int nr_cells = 0;
            Eigen::Affine3d T = pf.pcloud[i].GetAsAffine();
            double score = 1;

            if (ndts_filtered.size() == 0)
                fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
            Nn = ndts_filtered.size();

            for (int n = 0; n < ndts_filtered.size(); n++)
            {
                Eigen::Vector3d m = T * ndts_filtered[n]->getMean();
                if (m[2] < z_filter_min)
                    continue;

                perception_oru::NDTCell* cell;
                pcl::PointXYZL p;
                p.x = m[0];
                p.y = m[1];
                p.z = m[2];

                if (map_->getCellAtPoint(p, cell))
                {
                    // if(map.getCellForPoint(p,cell)){
                    if (cell == NULL)
                        continue;
                    if (cell->hasGaussian_)
                    {
                        Eigen::Matrix3d covCombined =
                            cell->getCov() +
                            T.rotation() * ndts_filtered[n]->getCov() * T.rotation().transpose();
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
                        nr_cells++;
                    }
                }
            }
            // score = nr_cells==0 ? 0 :score/nr_cells;
            pf.pcloud[i].SetLikelihood(score);
            // ROS_INFO_STREAM("Particle " << i << " score " << score);

            // totalHits += nr_cells;
            // if (nr_cells > maxHits)
            // {
            //     maxHits = nr_cells;
            // }
            //
        }
    } // #pragma

    // printPose(pf.getMean(), "   PF");

    t_pseudo = getDoubleTime() - t_pseudo;
    pf.normalize();

    // double hitp = (((double)totalHits / (double)pf.size()) / ndts.size()) * 100.0;
    // double bestp = ((double)maxHits / (double)ndts.size()) * 100.0;
    // std::cout << "loc hit%: " << hitp << std::endl;
    // std::cout << "top hit%: " << bestp << std::endl;
}

// ██████╗ ███████╗███████╗███████╗ █████╗ ██████╗  ██████╗██╗  ██╗
// ██╔══██╗██╔════╝██╔════╝██╔════╝██╔══██╗██╔══██╗██╔════╝██║  ██║
// ██████╔╝█████╗  ███████╗█████╗  ███████║██████╔╝██║     ███████║
// ██╔══██╗██╔══╝  ╚════██║██╔══╝  ██╔══██║██╔══██╗██║     ██╔══██║
// ██║  ██║███████╗███████║███████╗██║  ██║██║  ██║╚██████╗██║  ██║
// ╚═╝  ╚═╝╚══════╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝

//  FIXME: only for debugging
void MCLNDTType::ColorVoxelsWithScore(std::vector<perception_oru::NDTCell*> ndts)
{
    // ranging
    double maxScore = std::numeric_limits<double>::min();
    double minScore = std::numeric_limits<double>::max();
    std::map<int, double> scores;
    std::vector<double> scoresList;

    unsigned int Nn = 0;
    unsigned int nr_cells = 0;

    // this was particle pose
    Eigen::Affine3d T = pf.getMean();
    // printPose(T, "color");

    double score = 1;

    if (ndts.size() == 0)
        fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
    Nn = ndts.size();

    for (unsigned int n = 0; n < ndts.size(); n++)
    {
        Eigen::Vector3d m = T * ndts[n]->getMean();
        if (m[2] < z_filter_min)
        {
            scores[n] = -1;
            continue;
        }

        perception_oru::NDTCell* cell;
        pcl::PointXYZL p;
        p.x = m[0];
        p.y = m[1];
        p.z = m[2];

        if (map_->getCellAtPoint(p, cell))
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
                double voxelScore =
                    score_cell_weight + (1.0 - score_cell_weight) * exp(-0.05 * l / 2.0);
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

    // std::cout << "# points: " << scoresList.size() << std::endl;
    // std::cout << "min: " << minScore << " mean: " << mean << " max: " << maxScore <<
    // std::endl;
    // std::cout << "stdev: " << stdev << std::endl;

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
            b = 255.0f;
        }
        else
        {
            r = fmin(4 * (255.0f - score * 255.0f), 255.0f);
            g = fmin(4 * score * 255.0f, 255.0f);
        }

        cell->setRGB(r, g, b);
    }
    double sp = ((double)nr_cells / (double)Nn) * 100.0;
    // std::cout << "  color%: " << sp << std::endl;
}

std::vector<std::string> MCLNDTType::logPointHistogram(const std::string& path,
                                                       const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                                       const Eigen::Affine3d& pose,
                                                       int node_id)
{
    // create copy for access protection
    NDTMapPtr mapTypePtr =
        boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetNode(node_id)->GetMap());
    NDTMap* mapPtr = mapTypePtr->GetNDTMap();
    NDTMap map = *mapPtr;
    std::vector<std::string> strings;

    double sx, sy, sz;
    map.getGridSizeInMeters(sx, sy, sz);
    double maxDepth = max(sx, sy);
    Eigen::Vector3d origin(pose(0, 3), pose(1, 3), pose(2, 3));

    double cx, cy, cz;
    mapPtr->getCentroid(cx, cy, cz);
    std::cout << "global map origin: " << cx << " " << cy << " " << cz << std::endl;

    int successcount = 0;
    int labelsuccess = 0;
    int totalcount = cloud.points.size();

    // #pragma omp parallel for
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        pcl::PointXYZL point = cloud.points[i];
        Eigen::Vector3d direction(point.x, point.y, point.z);
        std::pair<double, int> pair = map.getDepthAndLabel(origin, direction, maxDepth);
        if (pair.first != -1)
        {
            successcount++;
            if (pair.second != 0)
            {
                labelsuccess++;
            }
        }

        double depth = direction.norm();
        double estimatedDepth = pair.first;
        int label = pair.second;

        std::stringstream ss;
        ss << point.label << ", " << label << ", " << depth << ", " << estimatedDepth << "; "
           << std::endl;
        std::string row = ss.str();
        // #pragma omp critical
        {
            strings.push_back(row);
        }
    }

    // std::cout << "Log histogram: " << successcount << "/" << labelsuccess << " out of " <<
    // totalcount << std::endl;
    std::cout << "success %: " << ((double)successcount / (double)totalcount) * 100.0 << std::endl;
    return strings;
}

void MCLNDTType::logMatchHistogram(const std::vector<perception_oru::NDTCell*>& ndts)
{
    Eigen::Affine3d T = pf.getMean();
    // printPose(T, "match");

    std::vector<std::string> strings;
    int hits = 0;

    int Nn = 0;
    unsigned int nr_cells = 0;

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

        if (map_->getCellAtPoint(p, cell))
        {
            if (cell == NULL)
                continue;
            if (cell->hasGaussian_)
            {
                int label = ndts[n]->getLabel();
                int mapLabel = cell->getLabel();
                double dist = ndts[n]->getMean().norm();
                double mapDist = cell->getMean().norm();
                if (mapLabel != -1)
                {
                    std::stringstream ss;
                    ss << label << ", " << mapLabel << ", " << dist << ", " << mapDist << std::endl;
                    strings.push_back(ss.str());
                    hits++;
                }
            }
        }
    }
    std::cout << "  match%: " << ((double)hits) / ((double)ndts.size()) * 100.0 << std::endl;

    {
        std::lock_guard<std::mutex> lock(histogram_mutex_);
        histogram_strings_.insert(histogram_strings_.end(), strings.begin(), strings.end());
    }
}

// ██████╗ ███████╗███████╗ █████╗ ███╗   ███╗██████╗ ██╗     ██╗███╗   ██╗ ██████╗
// ██╔══██╗██╔════╝██╔════╝██╔══██╗████╗ ████║██╔══██╗██║     ██║████╗  ██║██╔════╝
// ██████╔╝█████╗  ███████╗███████║██╔████╔██║██████╔╝██║     ██║██╔██╗ ██║██║  ███╗
// ██╔══██╗██╔══╝  ╚════██║██╔══██║██║╚██╔╝██║██╔═══╝ ██║     ██║██║╚██╗██║██║   ██║
// ██║  ██║███████╗███████║██║  ██║██║ ╚═╝ ██║██║     ███████╗██║██║ ╚████║╚██████╔╝
// ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝     ╚══════╝╚═╝╚═╝  ╚═══╝ ╚═════╝

double MCLNDTType::getPFProbabilityVariance() const
{
    double varP = 0;
    for (int i = 0; i < pf.size(); i++)
    {
        varP += (pf.pcloud[i].GetProbability() - 1.0 / pf.size()) *
                (pf.pcloud[i].GetProbability() - 1.0 / pf.size());
    }
    varP /= pf.size();
    varP = sqrt(varP);
    return varP;
}

Eigen::Matrix3d MCLNDTType::getPFPCovariance() const
{
    return pf.getCovariance();
}

Eigen::Vector3d MCLNDTType::getPFCloudSize() const
{
    Eigen::Matrix3d cov = getPFPCovariance();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov, false);
    return es.eigenvalues();
}

void MCLNDTType::SIResampling()
{
    if (SIRtype_ == 1)
    {
        double n_eff_particles = pf.getEffectiveParticleRatio();
        // std::cout << "n. eff: " << n_eff_particles << std::endl;
        if (n_eff_particles <= min_ratio_eff_particles_)
        {
            pf.SIRUpdate(n_particles_);
        }
    }
    else if (SIRtype_ == 2)
    {
        double varP = getPFProbabilityVariance();

        // std::cout << "varP: " << varP << std::endl;
        // std::cout << "sinceSIR_: " << sinceSIR_ << std::endl;

        if (varP > SIR_varP_threshold || sinceSIR_ > SIR_max_iters_wo_resampling_)
        {
            sinceSIR_ = 0;
            pf.SIRUpdate(n_particles_);
        }
        else
        {
            sinceSIR_++;
        }
    }
    else
    {
        pf.SIRUpdate(n_particles_);
    }
}

void MCLNDTType::ComputeMotionCovar(const Eigen::Affine3d& Tmotion,
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

//  ██████╗ ██████╗  ██████╗ ███╗   ███╗
// ██╔═══██╗██╔══██╗██╔═══██╗████╗ ████║
// ██║   ██║██║  ██║██║   ██║██╔████╔██║
// ██║   ██║██║  ██║██║   ██║██║╚██╔╝██║
// ╚██████╔╝██████╔╝╚██████╔╝██║ ╚═╝ ██║
//  ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝     ╚═╝

bool angleWarning(double a, const std::string& name, const Eigen::Vector3d& eul)
{
    if (fabs(a) > 0.05)
    {
        if (fabs(fabs(a) - 3.14) > 0.05)
        {
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            std::cout << name << " WARNING!" << std::endl;
            std::cout << eul.transpose() << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            return true;
        }
    }
    return false;
}

bool rollWarning(const Eigen::Vector3d& eul, const std::string& prefix)
{
    double rr = eul(0);
    double rp = eul(1);
    double ry = eul(2);
    bool a = angleWarning(rr, prefix + " ROLL", eul);
    bool b = angleWarning(rr, prefix + " PITCH", eul);
    return a or b;
}

void MCLNDTType::OdometryPrediction(const Eigen::Affine3d& Tmotion,
                                    bool disable_noise,
                                    pcl::PointCloud<pcl::PointXYZL>& cloud)
{

    Eigen::Matrix<double, 6, 1> motion_cov;
    if (!disable_noise)
    {
        ComputeMotionCovar(Tmotion, motion_cov);
    }
    else
    {
        motion_cov << 0, 0, 0, 0, 0, 0;
    }

    // Get mean estimate in world frame and find closest node
    // FIXME: max distance
    bool new_map_node = false;
    // std::cout << "since map change " << since_map_change_ << std::endl;
    if (!changed_map_ or since_map_change_ > map_change_hysteresis_)
    {
        new_map_node = graph_map_->SwitchToClosestMapNode(graph_map_->GetCurrentNodePose() *
                                                              pf.getMean() * sensor_pose_,
                                                          cloud, max_map_distance_, counter);
        changed_map_ = true;
    }
    if (new_map_node)
    {
        graph_map_->getNodeStates();
        Eigen::Affine3d Tprev_to_new;
        Tprev_to_new =
            (graph_map_->GetPreviousNodePose().inverse() * graph_map_->GetCurrentNodePose())
                .inverse();
        // cout << "change frame, transl=" << Tprev_to_new.translation().transpose() << endl;
        // cout << "                 rot=" << Tprev_to_new.rotation().eulerAngles(0, 1,
        // 2).transpose()
        //  << endl;
        pf.predict(Tmotion, motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3],
                   motion_cov[4], motion_cov[5], Tprev_to_new);
        map_ = this->GetCurrentNodeNDTMap();

        since_map_change_ = 0;
    }
    else
    {
        if (!enable_localisation_)
        {
            // Be aware numerical drift here, cannot be used with ground truth data
            pf.predict(Tmotion, 0, 0, 0, 0, 0, 0);
        }
        else
        {
            pf.predict(Tmotion, motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3],
                       motion_cov[4], motion_cov[5]);
        }
        since_map_change_++;
    }
}

// ██████╗ ██████╗ ███████╗██████╗ ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗
// ██╔══██╗██╔══██╗██╔════╝██╔══██╗██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║
// ██████╔╝██████╔╝█████╗  ██║  ██║██║██║        ██║   ██║██║   ██║██╔██╗ ██║
// ██╔═══╝ ██╔══██╗██╔══╝  ██║  ██║██║██║        ██║   ██║██║   ██║██║╚██╗██║
// ██║     ██║  ██║███████╗██████╔╝██║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║
// ╚═╝     ╚═╝  ╚═╝╚══════╝╚═════╝ ╚═╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝

bool MCLNDTType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor)
{
    bool updated = false;
    counter++;
    static Eigen::Affine3d Tmot_accum = Tmotion;

    if (counter > 0)
    {
        Tmot_accum = Tmot_accum * Tmotion;
    }

    if (!enable_localisation_)
    {
        // dont actually score the particles
        // disable prediction noise if no motion
        OdometryPrediction(Tmot_accum, true, cloud);

        if (createHistogram_)
        {
            if (histogramRatio_ == -1 || counter % histogramRatio_ == 0)
            {
                perception_oru::NDTMap local_map(
                    new perception_oru::LazyGrid(resolution * resolution_local_factor_));
                local_map.loadPointCloud(cloud);
                local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

                std::vector<perception_oru::NDTCell*> ndts_original = local_map.getAllCellsNoCopy();
                logMatchHistogram(ndts_original);
            }
        }

        pf.normalize();
    }
    else
    {
        // FIXME: this was Tmot_accum, but that's always zero
        OdometryPrediction(Tmotion, false, cloud);

        // else update particles
        perception_oru::NDTMap local_map(
            new perception_oru::LazyGrid(resolution * resolution_local_factor_));
        local_map.loadPointCloud(cloud);
        local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

        std::vector<perception_oru::NDTCell*> ndts_original = local_map.getAllCellsNoCopy();
        AssignParticleScore(ndts_original);

        SIResampling();

        if (display_local_map_)
        {
            if (display_error_map_)
            {
                ColorVoxelsWithScore(ndts_original);
                local_colors_ = 1;
            }
            NDTMap* mapPtr = &local_map;
            GraphPlot::PlotNDTRGB(mapPtr, "map", "local_map",
                                  graph_map_->GetCurrentNodePose() * pf.getMean(), local_colors_);
        }
        if (createHistogram_)
        {
            if (histogramRatio_ == -1 || counter % histogramRatio_ == 0)
            {
                logMatchHistogram(ndts_original);
            }
        }
    }

    if (lifelong_)
    {
        Eigen::Vector3d ev = getPFCloudSize();
        double th = lifelong_pf_size_th_;
        bool under = ev[0] < th && ev[1] < th && ev[2] < th;
        // std::cout << ev[0] << " " << ev[1] << " " << ev[2] << " " << std::endl;

        if (under)
        {
            Eigen::Affine3d tnow = pf.getMean();
            LifelongMapUpdate(cloud, tnow);
        }
        else
        {
            // note: debug
            std::cout << "Filter cloud size too large, do not perform lifelong update."
                      << std::endl;
        }
    }

    Tmot_accum = Eigen::Affine3d::Identity();
    SetPose(graph_map_->GetCurrentNodePose() * pf.getMean());

    if (visualize_)
    {
        geometry_msgs::PoseArray particles_msg =
            ParticlesToMsg(graph_map_->GetCurrentNodePose(), pf.pcloud, fix_frame_id);
        part_pub.publish(particles_msg);
    }
    updated = true;
    return updated;
}

// This is copied from fuser, should be refactored to single implementation
void MCLNDTType::LifelongMapUpdate(pcl::PointCloud<pcl::PointXYZL>& cloud, Eigen::Affine3d& Tnow)
{
    transformPointCloudInPlace(Tnow, cloud);

    Eigen::Vector3d localMapSize(130, 130, 20);
    graph_map_->m_graph.lock();

    map_->dbgClearUpdates();

    if(cluster_update_)
    {
        map_->addPointCloudClusterUpdate(Tnow.translation(), cloud, localMapSize,
                                        lifelong_max_numpoints_, lifelong_max_occupancy_,
                                        lifelong_max_z_, lifelong_sensor_noise_);
    }
    else
    {
        map_->addPointCloudMeanUpdate(Tnow.translation(), cloud, localMapSize,
                                        lifelong_max_numpoints_, lifelong_max_occupancy_,
                                        lifelong_max_z_, lifelong_sensor_noise_);
    }

    graph_map_->m_graph.unlock();
}

Eigen::Matrix3d MCLNDTType::getPFCovariance()
{
    return pf.getCovariance();
}

bool MCLNDTType::UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                  const Eigen::Affine3d& Tmotion,
                                  const Eigen::Affine3d& Tsensor)
{

    if (clouds.empty())
        return false;

    return UpdateAndPredict(clouds[0], Tmotion, Tsensor);
}

std::string MCLNDTType::ToString()
{

    std::stringstream ss;
    ss << LocalizationType::ToString();
    ss << graph_map_->ToString();
    ss << "NDTMCL:" << endl;
    ss << "number of particles: " << n_particles_ << endl;
    ss << "resolution: " << resolution << endl;
    ss << "resolution local_factor: " << resolution_local_factor_ << endl;
    ss << "force SIR: " << std::boolalpha << forceSIR << endl;
    ss << "SIR var Probability threshold: " << SIR_varP_threshold << endl;
    ss << "counter: " << counter << endl;
    ss << "sinceSIR_: " << sinceSIR_ << endl;
    ss << "ag: " << ag << endl;
    ss << "ratio_particles_initialize: " << ratio_particles_initialize << endl;
    ss << "SIR_max_iters_wo_resampling_: " << SIR_max_iters_wo_resampling_ << endl;
    ss << "initialized_: " << initialized_ << endl;
    ss << "subsample_level_: " << subsample_level_ << endl;
    ss << "z_filter_min: " << z_filter_min << endl;
    ss << "score_cell_weight: " << score_cell_weight << endl;
    ss << "createHistogram_: " << createHistogram_ << endl;
    ss << "histogramRatio_: " << histogramRatio_ << endl;
    ss << "local_map_set_: " << local_map_set_ << endl;
    ss << "display_error_map_: " << display_error_map_ << endl;
    ss << "display_local_map_: " << display_local_map_ << endl;
    ss << "distribution_error_: " << distribution_error_ << endl;
    ss << "max_map_distance_: " << max_map_distance_ << endl;
    ss << "motion model: ";
    ss << motion_model_.params.getDescString() << endl;
    for (int i = 0; i < motion_model_.params.motion_model.size(); i++)
    {
        if (i % 6 == 0)
        {
            ss << endl;
        }
        ss << motion_model_.params.motion_model(i) << " " << endl;
    }
    // ss << "motion model offset:";
    // for (int i = 0; i < motion_model_offset.size(); i++)
    // {
    //     if (i % 6 == 0)
    //     {
    //         ss << endl;
    //     };
    //     ss << motion_model_offset[i] << " " << endl;
    // }
    return ss.str();
}

// ██╗███╗   ██╗██╗████████╗██╗ █████╗ ██╗     ██╗███████╗███████╗
// ██║████╗  ██║██║╚══██╔══╝██║██╔══██╗██║     ██║╚══███╔╝██╔════╝
// ██║██╔██╗ ██║██║   ██║   ██║███████║██║     ██║  ███╔╝ █████╗
// ██║██║╚██╗██║██║   ██║   ██║██╔══██║██║     ██║ ███╔╝  ██╔══╝
// ██║██║ ╚████║██║   ██║   ██║██║  ██║███████╗██║███████╗███████╗
// ╚═╝╚═╝  ╚═══╝╚═╝   ╚═╝   ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚══════╝╚══════╝

void MCLNDTType::InitializeLocalization(const Eigen::Affine3d& pose,
                                        const Vector6d& variance,
                                        bool keep_current_cloud)
{
    // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
    std::cout << "initialize with variance: " << variance.transpose() << std::endl;
    SetPose(pose);
    pose_last_update_ = GetPose();

    bool new_map_node = graph_map_->SwitchToClosestMapNode(pose, max_map_distance_ * 50);

    map_ = this->GetCurrentNodeNDTMap();
    Eigen::Affine3d pose_local = graph_map_->GetCurrentNodePose().inverse() * pose;
    Eigen::Vector3d pos = pose_local.translation();
    Eigen::Vector3d euler = pose_local.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(euler);
    // particle filter is initialized with ratio_particles_initialize*n_particles_ particles,
    // however, after resampling, only n_particles_ particles should remaining
    pf.initializeUniformRandom(ratio_particles_initialize * n_particles_, pos(0), pos(1), pos(2),
                               euler(0), euler(1), euler(2), variance(0), variance(1), variance(2),
                               variance(3), variance(4), variance(5), keep_current_cloud);
    initialized_ = true;
}

// Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
void MCLNDTType::InitializeLocalization(const Eigen::Affine3d& pose,
                                        const Vector6d& variance,
                                        bool keep_current_cloud,
                                        int nr_part)
{
    // note: for histogramming
    if (!enable_localisation_)
    {
        Vector6d gt_var;
        gt_var << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // fairly certain
        InitializeLocalization(pose, gt_var, keep_current_cloud);
    }
    // default
    else
    {
        int tmp_nr_part = n_particles_;
        n_particles_ = nr_part;
        InitializeLocalization(pose, variance, keep_current_cloud);
        n_particles_ = tmp_nr_part;
    }
}

void MCLNDTType::InitializeLocalization(const Eigen::Affine3d& pose, const Vector6d& variance)
{ // Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
    InitializeLocalization(pose, variance, false);
}

MCLNDTParam::MCLNDTParam()
{
}

std::string MCLNDTParam::ToString()
{
    stringstream ss;
    ss << LocalisationParam::ToString();
    ss << "MCL-NDT parameters:" << endl;
    ss << "z_filter_min " << z_filter_min << endl;
    ss << "n_particles " << n_particles << endl;
    ss << "resolution_local_factor " << resolution_local_factor << endl;
    ss << "subsample_level " << subsample_level << endl;
    ss << "enable_localisation " << enable_localisation << endl;
    ss << "max_map_distance " << max_map_distance_ << endl;
    ss << "SIR_max_iters_wo_resampling " << SIR_max_iters_wo_resampling << endl;
    ss << "forceSIR " << std::boolalpha << forceSIR << endl;
    ss << "SIR_type " << SIRtype << endl;
    ss << "ratio_particles_initialize " << ratio_particles_initialize << std::endl;
    ss << "min_ratio_eff_particles " << min_ratio_eff_particles << std::endl;
    ss << "map_change_hysteresis " << map_change_hysteresis << std::endl;
    ss << "lifelong " << lifelong << std::endl;
    ss << "lifelong_pf_size_th " << lifelong_pf_size_th << std::endl;
    ss << "lifelong_max_numpoints " << lifelong_max_numpoints << std::endl;
    ss << "lifelong_max_occupancy " << lifelong_max_occupancy << std::endl;
    ss << "lifelong_max_z " << lifelong_max_z << std::endl;
    ss << "lifelong_sensor_noise " << lifelong_sensor_noise << std::endl;
    ss << "cluster_update " << cluster_update << std::endl;
    return ss.str();
}

void MCLNDTParam::GetParamFromRos()
{
    LocalisationParam::GetParamFromRos();
    cout << "MCLNDT localisation parameters from ROS" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("z_filter_height", z_filter_min, -10000.0);
    nh.param("particle_count", n_particles, 250);
    nh.param("resolution_local_factor", resolution_local_factor, 1.0);
    nh.param("subsample_level", subsample_level, 1.0);
    nh.param("SIR_max_iters_wo_resampling", SIR_max_iters_wo_resampling, 30);
    nh.param("force_SIR", forceSIR, false);
    nh.param("SIR_type", SIRtype, 0);
    nh.param("SIR_varP_threshold", SIR_varP_threshold, 0.5);
    nh.param("enable_localisation", enable_localisation, true);
    nh.param("max_map_distance", max_map_distance_, 50.0);
    nh.param("ratio_particles_initialize", ratio_particles_initialize, 10.0);
    nh.param("min_ratio_eff_particles", min_ratio_eff_particles, 0.5);
    nh.param("map_change_hysteresis", map_change_hysteresis, 5);
    nh.param("lifelong", lifelong, false);
    nh.param("lifelong_pf_size_th", lifelong_pf_size_th, 0.5);
    nh.param("lifelong_max_numpoints", lifelong_max_numpoints, 100000);
    nh.param("lifelong_max_occupancy", lifelong_max_occupancy, 255.0f);
    nh.param("lifelong_sensor_noise", lifelong_max_z, 20.0);
    nh.param("lifelong_sensor_noise", lifelong_sensor_noise, 0.06);
    nh.param("local_colors", local_colors, 1);
    nh.param("cluster_update", cluster_update, false);
    cout << "Fetched localisation parameters from ros" << endl;
    // cout<<ToString()<<endl;
}

} // namespace graph_localization
} // namespace perception_oru
