#ifndef UKF_NDT_H
#define UKF_NDT_H
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_localization_lamide/localization_type.h"
#include "graph_localization_lamide/ukf_ndt/3d_ndt_ukf.h"
#include "graph_localization_lamide/ukf_ndt/ukf_ndt_utils.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/motion_model_3d.h"
#include "pcl/point_cloud.h"
#include "stdio.h"
#include "time.h"
#include "vector"

#include <angles/angles.h>
#include <ndt_generic_lamide/point_types.h>


namespace perception_oru
{
namespace graph_localization
{

class UKFNDTType : public LocalizationType
{

public:
    UKFNDTType(LocalisationParamPtr param);

    ~UKFNDTType()
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

    void ComputeMotionCovar(const Eigen::Affine3d& Tmotion, Eigen::MatrixXd& motion_cov);

    NDTMap* GetCurrentNodeNDTMap();

    NDTMap* map_;
    Eigen::Affine3d pose_last_update_;
    // std::vector<double> motion_model, motion_model_offset;
    MotionModel3d motion_model_;
    ros::Publisher sigma_pub;
    int counter = 0; // ok
    double resolution = 0.5;
    double resolution_sensor = 0.5; // ok
    double voxel_filter_size = 1.;

    NDTUKF3D ukf;

private:
    friend class LocalisationFactory;
};
class UKFNDTParam : public LocalisationParam
{
public:
    UKFNDTParam();

    ~UKFNDTParam()
    {
    }

    void GetParamFromRos();

    std::string ToString();

    double range_var = 0.1;
    double min_pos_var = 0.1;
    double min_rot_var = 0.1;
    double range_filter_max_dist = 1.;
    int nb_ranges_in_update = 10000;
    int nb_updates = 1000;

    MotionModel3d motion_model;

private:
    friend class LocalisationFactory;
};
} // namespace graph_localization
} // namespace perception_oru

#endif // UKF_NDT_H
