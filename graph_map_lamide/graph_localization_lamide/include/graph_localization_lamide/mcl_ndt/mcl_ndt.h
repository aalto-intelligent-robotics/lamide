#ifndef MCL_NDT_H
#define MCL_NDT_H
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_localization_lamide/localization_type.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt_utils.h"
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

class MCLNDTType : public LocalizationType
{

public:
    MCLNDTType(LocalisationParamPtr param);

    ~MCLNDTType()
    {
    }

    std::string ToString();

    void InitializeLocalization(
        const Eigen::Affine3d& pose,
        const Vector6d& variance); // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    void InitializeLocalization(const Eigen::Affine3d& pose,
                                const Vector6d& variance,
                                bool keep_current_cloud); // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    void InitializeLocalization(const Eigen::Affine3d& pose,
                                const Vector6d& variance,
                                bool keep_current_cloud,
                                int nr_part); // Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

    bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZL>& cloud,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor);

    bool UpdateAndPredict(std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                          const Eigen::Affine3d& Tmotion,
                          const Eigen::Affine3d& Tsensor);

    void GetParamFromRos();

    // This assigns rgb color according to the weighted score
    void ColorVoxelsWithScore(std::vector<perception_oru::NDTCell*> ndts) override;

    std::vector<std::string> logPointHistogram(const std::string& path,
                                               const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                               const Eigen::Affine3d& pose,
                                               int node_id) override;

    void logMatchHistogram(const std::vector<perception_oru::NDTCell*>& ndts) override;

    std::vector<std::string> getHistogramStrings() override
    {
        std::lock_guard<std::mutex> lock(histogram_mutex_);
        return histogram_strings_;
    }

    void setCreateHistogram(bool set, int ratio = -1) override
    {
        createHistogram_ = set;
        histogramRatio_ = ratio;
    }

    void setDisplayErrorMap(bool setError, bool setLocal) override
    {
        display_error_map_ = setError;
        display_local_map_ = setLocal;
    }

    void setDistributionError(bool set) override
    {
        distribution_error_ = set;
    }

    void SetMapSwitchMethod(MapSwitchingMethod method) override
    {
        graph_map_->SetMapSwitchMethod(method);
    }

    void SetMapInterchangeRadius(double radius) override
    {
        graph_map_->SetMapInterchangeRadius(radius);
    }

    MapSwitchingMethod GetMapSwitchMethod() const override
    {
        return graph_map_->GetMapSwitchMethod();
    }

    perception_oru::NDTMap& getLocalMap(bool& success) override
    {
        if (local_map_set_)
        {
            success = true;
        }
        else
        {
            success = false;
        }
        std::lock_guard<std::mutex> lock(local_map_mutex_);
        return local_map_;
    }

    Eigen::Matrix3d getPFCovariance();

protected:
    void OdometryPrediction(const Eigen::Affine3d& Tmotion,
                            bool disable_noise,
                            pcl::PointCloud<pcl::PointXYZL>& cloud);

    void ComputeMotionCovar(const Eigen::Affine3d& Tmotion,
                            Eigen::Matrix<double, 6, 1>& motion_cov);

    virtual void AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts);

    void SIResampling();

    NDTMap* GetCurrentNodeNDTMap();

    void LifelongMapUpdate(pcl::PointCloud<pcl::PointXYZL>& cloud, Eigen::Affine3d& Tnow);

    double getPFProbabilityVariance() const;

    Eigen::Matrix3d getPFPCovariance() const;

    Eigen::Vector3d getPFCloudSize() const;

    particle_filter_3d pf;
    NDTMap* map_;
    Eigen::Affine3d pose_last_update_;
    // std::vector<double> motion_model, motion_model_offset;
    MotionModel3d motion_model_;
    ros::Publisher part_pub;
    int counter = 0;   // ok
    int sinceSIR_ = 0; // ok
    int SIRtype_ = 0;
    int ag = 250;
    double ratio_particles_initialize = 10; // number of particles during initialization
    int n_particles_ = 250;                 // number of particles during continuous operation
    int SIR_max_iters_wo_resampling_ = 25;
    bool initialized_ = false;
    bool forceSIR = false; // ok
    double resolution = 0.5;
    double resolution_local_factor_ = 1.0; // ok
    double subsample_level_ = 1.0;
    double z_filter_min = -10000.0;
    double score_cell_weight = 0.1;
    double SIR_varP_threshold = 0.6;
    std::vector<std::string> histogram_strings_;
    bool createHistogram_ = false;
    int histogramRatio_ = -1;
    std::mutex histogram_mutex_;
    std::mutex local_map_mutex_;
    perception_oru::NDTMap local_map_;
    bool local_map_set_ = false;
    bool display_error_map_ = false;
    bool display_local_map_ = false;
    bool distribution_error_ = false;
    double max_map_distance_ = 50;
    double min_ratio_eff_particles_ = 0.5;
    int since_map_change_ = 0;
    bool changed_map_ = false;
    int map_change_hysteresis_ = 5;
    bool lifelong_ = false;
    double lifelong_pf_size_th_ = 0.5;
    unsigned int lifelong_max_numpoints_ = 1e5;
    float lifelong_max_occupancy_ = 255.0f;
    double lifelong_max_z_ = 20.0;
    double lifelong_sensor_noise_ = 0.06;
    int local_colors_ = 1;
    bool cluster_update_ = false;

private:
    friend class LocalisationFactory;
};
class MCLNDTParam : public LocalisationParam
{
public:
    MCLNDTParam();

    ~MCLNDTParam()
    {
    }

    virtual void GetParamFromRos();

    std::string ToString();

    bool forceSIR = false;
    int SIRtype = 0;
    double z_filter_min = -10000.0;
    double score_cell_weight = 0.1;
    double SIR_varP_threshold = 0.6;
    int n_particles = 250;
    int SIR_max_iters_wo_resampling = 30;
    double subsample_level = 1.0;
    MotionModel3d motion_model;
    double resolution_local_factor = 1.0;
    double max_map_distance_ = 50;
    double ratio_particles_initialize = 10;
    double min_ratio_eff_particles = 0.5;
    int map_change_hysteresis = 5;
    bool lifelong = false;
    double lifelong_pf_size_th = 0.5;
    int lifelong_max_numpoints = 1e5;
    float lifelong_max_occupancy = 255.0f;
    double lifelong_max_z = 20.0;
    double lifelong_sensor_noise = 0.06;
    int local_colors = 1;
    bool cluster_update = false;
    // std::vector<double> motion_model, motion_model_offset;
private:
    friend class LocalisationFactory;
};
} // namespace graph_localization
} // namespace perception_oru
#endif // MCL_NDT_H
