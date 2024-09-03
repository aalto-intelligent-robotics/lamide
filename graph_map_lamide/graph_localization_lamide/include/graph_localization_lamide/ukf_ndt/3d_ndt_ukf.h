#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cstdio>
#include <fstream>
#include <ndt_generic_lamide/eigen_utils.h>
#include <ndt_generic_lamide/pcl_utils.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/pointcloud_utils.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf_conversions/tf_eigen.h>


//! Implements a UKF in 3D (using x,y,z and euler angles)
class UKF3D
{
public:
    class Params
    {
    public:
        Params()
            : alpha(0.001),
              beta(2.),
              kappa(0.),
              range_var(0.1),
              min_pos_var(0.1),
              min_rot_var(0.1),
              range_filter_max_dist(1.),
              nb_ranges_in_update(100),
              nb_updates(1000),
              debug(false)

        {
        }
        double alpha;
        double beta;
        double kappa;
        double range_var;
        double min_pos_var;
        double min_rot_var;
        double range_filter_max_dist;
        int nb_ranges_in_update;
        int nb_updates;
        bool debug;

        friend std::ostream& operator<<(std::ostream& os, const UKF3D::Params& obj)
        {
            os << "\nalpha                 : " << obj.alpha;
            os << "\nbeta                  : " << obj.beta;
            os << "\nkappa                 : " << obj.kappa;
            os << "\nrange_var             : " << obj.range_var;
            os << "\nmin_pos_var           : " << obj.min_pos_var;
            os << "\nmin_rot_var           : " << obj.min_rot_var;
            os << "\nrange_filter_max_dist : " << obj.range_filter_max_dist;
            os << "\nnb_ranges_in_update   : " << obj.nb_ranges_in_update;
            os << "\nnb_updates            : " << obj.nb_updates;
            os << "\ndebug                 : " << obj.debug;
            return os;
        }
    };

    UKF3D() : N_(6)
    {

        allocateSigmas();
    }

    void setParams(const UKF3D::Params& params)
    {
        params_ = params;
    }

    std::string getDebugString() const
    {
        std::ostringstream stream;
        stream << "alpha : " << params_.alpha << " beta : " << params_.beta
               << " kappa : " << params_.kappa << " N : " << N_
               << " # sigma points : " << sigmas_.size() << std::endl;
        for (int i = 0; i < sigmas_.size(); i++)
        {
            stream << "[" << i << "] : " << sigmas_[i].transpose() << std::endl;
        }
        return stream.str();
    }

    int getNbSigmaPoints() const
    {
        return 2 * N_ + 1;
    }

    std::vector<double> getMeanWeights() const;
    std::vector<double> getCovWeights() const;

    Eigen::Affine3d getMean() const
    {

        return T_;
    }

    Eigen::MatrixXd getCov() const
    {
        return P_;
    }

    Eigen::VectorXd computePoseMean() const;
    Eigen::MatrixXd computePoseCov(const Eigen::VectorXd& mean) const;

    double getWeightMean(int idx) const;
    double getWeightCov(int idx) const;

    // Sigma points, stored as x,y,z, roll,pitch, yaw.
    std::vector<Eigen::VectorXd> sigmas_;
    // Current pose estimate
    Eigen::Affine3d T_;
    // Current pose covariance
    Eigen::MatrixXd P_;
    // UKF parameters
    Params params_;
    // State dim
    int N_;

    double getLambda() const
    {
        return params_.alpha * params_.alpha * (N_ + params_.kappa) - N_;
    }

    double getXsi() const
    {
        return sqrt(N_ + getLambda());
    }

    const Eigen::VectorXd& getSigma(int i) const
    {
        return sigmas_[i];
    }

    void allocateSigmas()
    {
        // assert(sigmas_.empty()); //how could there even be s-points by the time its created?
        sigmas_.clear();
        for (int i = 0; i < 2 * N_ + 1; i++)
        {
            Eigen::VectorXd s(N_);
            sigmas_.push_back(s);
        }
    }

    std::vector<Eigen::Affine3d> getSigmasAsAffine3d() const;

    void assignSigmas(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

    void initializeFilter(const Eigen::Affine3d& pose, const Eigen::MatrixXd& posecov)
    {
        T_ = pose;
        P_ = posecov;

        assignSigmas(ndt_generic::affine3dToVector(T_), P_);
    }

    void normalizeEuler()
    {
        for (int i = 0; i < sigmas_.size(); i++)
        {
            ndt_generic::normalizeEulerAngles6dVec(sigmas_[i]);
        }
    }

    void predict(const Eigen::Affine3d& incr, const Eigen::MatrixXd& incrCov);

    // Assure that the diagonal vairances is never less than ...
    void assignMinDiagVariances();

    // Filter the predicted and raw readings to assure that the distance between them are less than
    // a threshold.
    std::vector<int> filterMeasurements(Eigen::MatrixXd& pred_ranges,
                                        Eigen::VectorXd& raw_ranges,
                                        Eigen::MatrixXd& filter_pred_ranges,
                                        Eigen::VectorXd& filter_raw_ranges) const;

    // The update step (fuse the sensor readings and predicted readings)
    void update(const Eigen::MatrixXd& pred_ranges, const Eigen::VectorXd& raw_ranges);

    // Update with a global pose and covariance
    void update(const Eigen::Affine3d& pose, const Eigen::MatrixXd& cov);

    // Divides the sensory readings / prediction and performs a set of udpate steps.
    void updateSeq(const Eigen::MatrixXd& pred_ranges, const Eigen::VectorXd& raw_ranges);
};

/**
 * NDT UKF 3D - Class implementation, puts the NDT map and UKF together.
 */
class NDTUKF3D
{
public:
    /**
     * Constructor
     */
    NDTUKF3D()
    {
        isInit = false;
        counter = 0;
        map_ = NULL;
        map_offset_.setIdentity();
    }

    void computeMeasurements(pcl::PointCloud<pcl::PointXYZL>& cloud,
                             const Eigen::Affine3d& Tsensor,
                             Eigen::MatrixXd& meassurements,
                             Eigen::VectorXd& raw_ranges);

    void predict(Eigen::Affine3d Tmotion, const Eigen::MatrixXd& incrCov);

    void update(pcl::PointCloud<pcl::PointXYZL>& cloud, const Eigen::Affine3d& Tsensor);

    void update(const Eigen::Affine3d& pose, const Eigen::MatrixXd& cov);

    //    void updateAndPredictEff(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZL> &cloud,
    //    double subsample_level);

    Eigen::Affine3d getMean() const
    {
        return ukf_.getMean();
    }

    void initializeFilter(const Eigen::Affine3d& T,
                          double varx,
                          double vary,
                          double varz,
                          double varR,
                          double varP,
                          double varY)
    {
        Eigen::MatrixXd cov(6, 6);
        cov.setZero();
        cov(0, 0) = varx;
        cov(1, 1) = vary;
        cov(2, 2) = varz;
        cov(3, 3) = varR;
        cov(4, 4) = varP;
        cov(5, 5) = varY;

        ukf_.initializeFilter(T, cov);
    }

    std::vector<Eigen::Affine3d> getSigmasAsAffine3d() const
    {
        return ukf_.getSigmasAsAffine3d();
    }

    void setParamsUKF(const UKF3D::Params& params)
    {
        ukf_.setParams(params);
    }

    const UKF3D::Params& getParamsUKF()
    {
        return ukf_.params_;
    }

    void updateVisualizationClouds(const std::vector<int>& idx,
                                   const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                   const Eigen::MatrixXd& filter_pred_ranges,
                                   const Eigen::Affine3d& Tsensor);

    pcl::PointCloud<pcl::PointXYZL>& getFilterRaw()
    {
        return pc_filtered_raw_;
    }

    pcl::PointCloud<pcl::PointXYZL>& getFilterPred()
    {
        return pc_filtered_pred_;
    }

    // An affine offset between the reported pose and the current map (used if mulitple maps with
    // different origins are used).
    void setMapOffset(const Eigen::Affine3d& offset);

    void setMap(perception_oru::NDTMap* map)
    {
        map_ = map;
    }

private:
    bool isInit;
    double getDoubleTime()
    {
        struct timeval time;
        gettimeofday(&time, NULL);
        return time.tv_sec + time.tv_usec * 1e-6;
    }

    // UKF variables
    UKF3D ukf_;

    // Store some point clouds for debugging / visualization
    pcl::PointCloud<pcl::PointXYZL> pc_filtered_raw_;
    pcl::PointCloud<pcl::PointXYZL> pc_filtered_pred_;

    perception_oru::NDTMap* map_;
    Eigen::Affine3d map_offset_;
    int counter;
    std::vector<Eigen::Vector3d> dirs_in_vehicle_frame_; // Only used for visualization
};
