#include "graph_localization_lamide/mcl_ndt/mcl_ndt_lamide.h"

#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/utils.h"
#include <limits>
#include <ndt_generic_lamide/math_utils.h>

namespace perception_oru
{
namespace graph_localization
{

MCLNDTLamideType::MCLNDTLamideType(LocalisationParamPtr param)
    : MCLNDTType(param)
{
    if (MCLNDTLamideParamPtr mclParam = boost::dynamic_pointer_cast<MCLNDTLamideParam>(param))
    {
        parseWeights(mclParam->weights_path_);
    }
    else
    {
        std::cerr << "Cannot create MCLNDType. Illegal type for parameter param" << endl;
        exit(0);
    }
}

void MCLNDTLamideType::parseWeights(const std::string& path)
{
    std::ifstream file;
    file.open(path);
    std::string line;
    const std::string delimiter = ",";
    if (file.is_open())
    {
        while (getline(file, line))
        {
            size_t pos = 0;
            std::vector<std::string> tokens;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                std::string token = line.substr(0, pos);
                tokens.push_back(token);
                line.erase(0, pos + delimiter.length());
            }
            tokens.push_back(line);
            if (tokens.size() == 3)
            {
                int label1 = std::stoi(tokens[0]);
                int label2 = std::stoi(tokens[1]);
                double weight = std::stod(tokens[2]);
                weights_.set(label1, label2, weight);
            }
        }
        file.close();
    }
    weights_.print();
}

void MCLNDTLamideType::AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts)
{
    // debug
    double div_max = -1;
    int div_n = 0;
    double div_sum = 0;
    // end

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

                        double cellScore =
                            score_cell_weight + (1.0 - score_cell_weight) * exp(-0.05 * l / 2.0);

                        // TODO: specialization
                        double semantic_coefficient;
                        if (distribution_error_)
                        {
                            Eigen::ArrayXd map_labels = cell->getLabelDistribution();
                            Eigen::ArrayXd local_labels = ndts_filtered[n]->getLabelDistribution();
                            double div = KLDivergence(local_labels, map_labels);
                            semantic_coefficient = weightingFunction(div);

                            if (semantic_coefficient > div_max)
                            {
                                div_max = semantic_coefficient;
                            }
                            div_n++;
                            div_sum += semantic_coefficient;
                        }
                        else
                        {
                            semantic_coefficient =
                                weights_.get(ndts_filtered[n]->getLabel(), cell->getLabel());
                        }

                        cellScore *= semantic_coefficient;
                        score += cellScore;

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

    double div_mean = div_sum / (double)div_n;
    std::cout << "max LaMiDE coefficient: " << div_max << std::endl;
    std::cout << "mean LaMiDE coefficient: " << div_mean << std::endl;

    // double hitp = (((double)totalHits / (double)pf.size()) / ndts.size()) * 100.0;
    // double bestp = ((double)maxHits / (double)ndts.size()) * 100.0;
    // std::cout << "loc hit%: " << hitp << std::endl;
    // std::cout << "top hit%: " << bestp << std::endl;
}

void MCLNDTLamideParam::GetParamFromRos()
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
    nh.param("weights_path", weights_path_, std::string(""));
    nh.param("max_map_distance", max_map_distance_, 50.0);
    nh.param("ratio_particles_initialize", ratio_particles_initialize, 10.0);
    nh.param("min_ratio_eff_particles", min_ratio_eff_particles, 0.5);
    cout << "Fetched localisation parameters from ros" << endl;
    // cout<<ToString()<<endl;
}

} // namespace graph_localization
} // namespace perception_oru
