#ifndef UKF_REG_H
#define UKF_REG_H
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_localization_lamide/localization_type.h"
#include "graph_localization_lamide/ukf_ndt/3d_ndt_ukf.h"
#include "graph_localization_lamide/ukf_ndt/ukf_ndt_utils.h"
#include "graph_map_lamide/ndt/ndt_map_param.h"
#include "graph_map_lamide/ndt/ndt_map_type.h"
#include "graph_map_lamide/ndt/ndtd2d_reg_type.h"
#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/ndt_dl/ndtdl_reg_type.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/motion_model_3d.h"
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

class UKFRegType : public LocalizationType
{

public:
    UKFRegType(LocalisationParamPtr param);

    ~UKFRegType()
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

    unsigned int GetFramesCount()
    {
        return regptr_->GetRegistrationCount();
    }

protected:
    void OdometryPrediction(const Eigen::Affine3d& Tmotion, bool disable_noise);

    void ComputeMotionCovar(const Eigen::Affine3d& Tmotion, Eigen::MatrixXd& motion_cov);

    NDTMap* GetCurrentNodeNDTMap();

    NDTMap* map_;
    Eigen::Affine3d pose_last_update_;
    MotionModel3d motion_model_;
    ros::Publisher sigma_pub;
    int counter = 0; // ok

    RegTypePtr regptr_;
    NDTUKF3D ukf;
    std::vector<double> min_obs_variance_;
    bool force_use_registration_;

private:
    void UpdateMinObservationCov(Eigen::MatrixXd& cov);

    friend class LocalisationFactory;
};

class UKFRegParam : public LocalisationParam
{
public:
    UKFRegParam();

    ~UKFRegParam()
    {
    }

    void GetParamFromRos();

    std::string ToString();

    double min_pos_var = 0.1;
    double min_rot_var = 0.1;
    RegParamPtr registration_parameters;

    MotionModel3d motion_model;
    std::vector<double> min_obs_variance;
    bool force_use_registration = false;

private:
    friend class LocalisationFactory;
};
} // namespace graph_localization
} // namespace perception_oru

#endif // UKF_NDT_H
