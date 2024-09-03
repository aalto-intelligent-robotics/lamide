#ifndef SUBMAP_MCL_H
#define SUBMAP_MCL_H
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_localization_lamide/localization_type.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt_utils.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//#include "ndt_localization_lamide/3d_ndt_mcl.hpp"
#include "boost/shared_ptr.hpp"
#include "graph_localization_lamide/LocalisationHeatMap.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/motion_model_3d.h"
#include "ndt_localization_lamide/3d_particle_filter.hpp"
#include "stdio.h"
#include "time.h"
#include "vector"

#include <angles/angles.h>
#include <ndt_generic_lamide//point_types.h>
#include <tuple>


using namespace HeatMap;
namespace perception_oru
{
namespace graph_localization
{

class SubmapMCLType : public LocalizationType
{

public:
    SubmapMCLType(LocalisationParamPtr param);

    ~SubmapMCLType()
    {
    }

    std::string ToString();

    void InitializeLocalization(
        const Eigen::Affine3d& pose,
        const Vector6d& variance); // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    void InitializeLocalization(const Eigen::Affine3d& pose,
                                const Vector6d& variance,
                                bool keep_current_cloud); // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    void DelayedInitialization(
        const Eigen::Affine3d& pose,
        const Vector6d&
            variance); // Samples particles, skips the prediction and then fuses the initialization.
                       // This is usefull to efficiently integrate the particles.

    void UniformInitialization(const Vector6d& spread, const Vector6d& resolution);

    // void DelayedInitialization(Eigen::MatrixXd &cov, const Eigen::Matrix<double, 6, 1> &scaling,
    // const Eigen::Affine3d &pose);

    bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor);

    bool UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor);

    bool Reset();

    void GetParamFromRos();

protected:
    void GenerateSpreadRot(pcl::PointCloud<pcl::PointXYZL>::Ptr& points,
                           const Vector6d& spread,
                           const Vector6d& resolution,
                           ndt_generic::Affine3dSTLVek& Poses);

    void GenerateSpreadPos(const Eigen::Vector3d& position,
                           const Vector6d& spread,
                           const Vector6d& resolution,
                           pcl::PointCloud<pcl::PointXYZL>::Ptr cloud);

    void OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise);

    void ComputeMotionCovar(const Eigen::Affine3d& Tmotion,
                            Eigen::Matrix<double, 6, 1>& motion_cov);

    void AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts);

    void SIResampling();

    void Resample();

    void init(const Eigen::Affine3d& pose,
              const Vector6d& variance,
              particle_filter_3d& part,
              bool keep_current_cloud);

    NDTMap* GetCurrentNodeNDTMap(MapNodePtr nodePtr);

    particle_filter_3d pf, pf_delayed;

    Eigen::Affine3d pose_last_update_;
    // std::vector<double> motion_model, motion_model_offset;
    MotionModel3d motion_model_;
    ros::Publisher part_pub;
    int counter = 0;                       // ok
    int sinceSIR_ = 0;                     // ok
    double ratio_particles_initialize = 1; // number of particles during initialization
    int n_particles_ = 250;                // number of particles during continuous operation
    int SIR_max_iters_wo_resampling_ = 25;
    bool initialized_ = false;
    bool forceSIR = false; // ok
    double resolution = 0.5;
    double resolution_local_factor_ = 1.0; // ok
    double subsample_level_ = 1.0;
    double z_filter_min = -10000.0;
    double score_cell_weight = 0.1;
    double SIR_varP_threshold = 0.6;
    float percent_inliers = 1.0;
    boost::shared_ptr<HeatMapInterface> heatmap_;
    Eigen::Affine3d Tfirst_;
    std::string heatmap_file_path_ = "";
    bool load_previous_heatmap_ = true, save_heatmap_ = false;
    bool uniform_initialization_ = false;
    float convergance_rate_ =
        0.5; // speed of convergance, 1 = stagnation, 0 = immidiate level convergance

private:
    friend class LocalisationFactory;
};
class SubmapMCLParam : public LocalisationParam
{
public:
    SubmapMCLParam();

    ~SubmapMCLParam()
    {
    }

    void GetParamFromRos();

    std::string ToString();

    bool forceSIR = false;
    double z_filter_min = -10000.0;
    double score_cell_weight = 0.1;
    double SIR_varP_threshold = 0.6;
    int n_particles = 250;
    bool uniform_initialization = false;
    float convergance_rate = 0.5;
    int SIR_max_iters_wo_resampling = 30;
    MotionModel3d motion_model;
    double resolution_local_factor = 1.0;
    std::string heatmap_file_path = "";
    bool load_previous_heatmap = false, save_heatmap = false;
    // std::vector<double> motion_model, motion_model_offset;
private:
    friend class LocalisationFactory;
};
} // namespace graph_localization
} // namespace perception_oru
#endif // SUBMAP_MCL_H
