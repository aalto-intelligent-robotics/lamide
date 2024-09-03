#ifndef REGISTRATIONTYPE_H
#define REGISTRATIONTYPE_H
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"
#include "graph_map_lamide/map_type.h"
#include "graphfactory.h"
#include "iostream"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_registration_lamide/registration.h"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "stdio.h"
#include "string"

#include <ndt_generic_lamide/point_types.h>


using std::cout;
using std::endl;
namespace perception_oru
{
namespace graph_map
{

class registrationParameters
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~registrationParameters() = 0;

    virtual void GetParametersFromRos();

    bool enable_registration = true;
    bool registration2d = false;
    bool do_soft_constraints = false;
    bool calculate_cov = false;
    bool check_consistency = true;
    bool use_initial_guess = true;
    double max_translation_norm = 0.3, max_rotation_norm = M_PI / 10.0;
    double translation_registration_delta = 0, rotation_registration_delta = 0;
    double map_size_z = 12;
    double sensor_range = 130;
    Eigen::Affine3d sensor_pose;
    // lslgeneric::MotionModel2d motion_model_2d_;
    //  lslgeneric::MotionModel3d motion_model_3d_;
protected:
    registrationParameters();

private:
    friend class GraphFactory;
};

class registrationType
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~registrationType() = 0;

    /*!
     * \brief RegisterScan Perform scan-to-map registration
     * \param maptype The map in the selected scan-to-map registration method
     * \param Tnow Prediction and registration result
     * \param cloud pointcloud
     * \param Tcov Covariance input and/or output
     * \return false only if registration was performed and failed
     */
    bool RegisterScan(MapTypePtr maptype,
                      Eigen::Affine3d& Tnow,
                      pcl::PointCloud<pcl::PointXYZL>& cloud,
                      Eigen::MatrixXd& Tcov);

    bool RegisterScan(MapTypePtr maptype,
                      Eigen::Affine3d& Tnow,
                      pcl::PointCloud<pcl::PointXYZL>& cloud);

    bool RegisterScan(MapTypePtr maptype,
                      Eigen::Affine3d& Tnow,
                      std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                      Eigen::MatrixXd& Tcov);

    bool RegisterScan(MapTypePtr maptype,
                      Eigen::Affine3d& Tnow,
                      std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds);

    bool RegisterScan(pcl::PointCloud<pcl::PointXYZL>& target,
                      Eigen::Affine3d& Tnow,
                      pcl::PointCloud<pcl::PointXYZL>& src,
                      Eigen::MatrixXd& Tcov);

    bool Map2MapRegistration(MapTypePtr target,
                             MapTypePtr source,
                             Eigen::Affine3d& Tdiff,
                             double& match_score);

    virtual std::string ToString();

    void visualize(bool viz, const Eigen::Affine3d& offset)
    {
        offset_ = offset;
        visualize_ = viz;
    }

    const double GetScore() const
    {
        return score_;
    }

    unsigned int GetRegistrationCount()
    {
        return failed_registrations + succesfull_registrations;
    }

    bool enable_registraiton = true;
    bool registration2d_ = true;
    bool do_soft_constraints_ = false;
    bool calculate_cov_ = false;
    bool check_consistency = true;
    bool use_initial_guess = true;
    double max_translation_norm = 0.6, max_rotation_norm = 3 * M_PI / 180;
    double translation_registration_delta = 0, rotation_registration_delta = 0;
    double sensor_range = 100;
    double map_size_z;
    unsigned int failed_registrations = 0;
    unsigned int succesfull_registrations = 0;
    Eigen::Affine3d sensorPose_ =
        Eigen::Affine3d::Identity(); // Translation between robot and sensor frame
    Eigen::MatrixXd OutputCov;

protected:
    registrationType(RegParamPtr regparam);

    virtual bool Register(MapTypePtr maptype,
                          Eigen::Affine3d& Tnow,
                          pcl::PointCloud<pcl::PointXYZL>& cloud,
                          Eigen::MatrixXd& Tcov)
    {
        std::cerr << "No registration for pcl::PointXYZL implemented" << endl;
    } // This methods attempts to register the point cloud versus the map using the affine
      // transformation guess "Tm"

    virtual bool Register(MapTypePtr maptype,
                          Eigen::Affine3d& Tnow,
                          std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                          Eigen::MatrixXd& Tcov)
    {
        std::cerr << "No registration for std::vector<pcl::PointXYZL> implemented" << endl;
    }

    virtual bool RegisterMap2Map(MapTypePtr target,
                                 MapTypePtr source,
                                 Eigen::Affine3d& Tdiff,
                                 double& match_score)
    {
    }

    virtual bool Register(pcl::PointCloud<pcl::PointXYZL>& target,
                          Eigen::Affine3d& Tnow,
                          pcl::PointCloud<pcl::PointXYZL>& src,
                          Eigen::MatrixXd& Tcov)
    {
        std::cerr << "No registration for std::vector<pcl::PointXYZL> implemented" << endl;
    }

    bool visualize_ = false;
    Eigen::Affine3d offset_ = Eigen::Affine3d::Identity();
    regStatus status_ = SUCCESS; // Contain the status of the registration outcome
    double score_ = 0;

private:
    friend class GraphFactory;
};

} // namespace graph_map

} // namespace perception_oru
#endif // REGISTRATIONTYPE_H
