#ifndef MCL_NDTDL_H
#define MCL_NDTDL_H
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_localization_lamide/localization_type.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/motion_model_3d.h"
#include "ndt_localization_lamide/3d_ndt_mcl.hpp"
#include "ndt_localization_lamide/3d_particle_filter.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "stdio.h"
#include "time.h"
#include "vector"

#include <angles/angles.h>
#include <ndt_generic_lamide/point_types.h>


namespace perception_oru
{
namespace graph_localization
{

class MCLNDTDLType : public LocalizationType
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MCLNDTDLType(LocalisationParamPtr param);

    ~MCLNDTDLType()
    {
    }

    std::string ToString();

    void InitializeLocalization(
        const Eigen::Affine3d& pose,
        const Vector6d& variance); // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor);

    bool UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor);

    void GetParamFromRos();

protected:
    void OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise);

    void ComputeMotionCovar(const Eigen::Affine3d& Tmotion,
                            Eigen::Matrix<double, 6, 1>& motion_cov);

    void AssignParticleScore(std::vector<std::vector<perception_oru::NDTCell*>> ndts);

    void SIResampling();

    std::vector<NDTMap*> GetCurrentNodeNDTMaps();

    particle_filter_3d pf;
    std::vector<NDTMap*> maps_;
    Eigen::Affine3d pose_last_update_;
    // std::vector<double> motion_model, motion_model_offset;
    MotionModel3d motion_model_;
    ros::Publisher part_pub;
    int counter = 0;   // ok
    int sinceSIR_ = 0; // ok
    int n_particles_ = 250;
    int SIR_max_iters_wo_resampling_ = 25;
    bool initialized_ = false;
    bool forceSIR = false; // ok
    double resolution = 0.5;
    double resolution_sensor = 0.5; // ok
    double subsample_level_ = 1.0;
    double z_filter_min = -10000.0;
    double score_cell_weight = 0.1;
    double SIR_varP_threshold = 0.6;
    bool visualize_sensor_maps_ = true;

private:
    friend class LocalisationFactory;
};
class MCLNDTDLParam : public LocalisationParam
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MCLNDTDLParam();

    ~MCLNDTDLParam()
    {
    }

    void GetParamFromRos();

    std::string ToString();

    bool forceSIR = false;
    double z_filter_min = -10000.0;
    double score_cell_weight = 0.1;
    double SIR_varP_threshold = 0.6;
    int n_particles = 250;
    int SIR_max_iters_wo_resampling = 30;
    MotionModel3d motion_model;
    // std::vector<double> motion_model, motion_model_offset;
private:
    friend class LocalisationFactory;
};

} // namespace graph_localization
} // namespace perception_oru
#endif // MCL_NDTDL_H
