#ifndef NDT_GENERIC_MOTION_MODELS_3D_H
#define NDT_GENERIC_MOTION_MODELS_3D_H

#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <iostream>

#include <ndt_generic_lamide/eigen_utils.h>
#include <ndt_generic_lamide/motion_model_2d.h>

//! Motion model (incremental).
/*!
  The only motion model that are really useful is the relative  incremental one.
  That is, given two measurement at t0 and t1, obtain the relative
  odometry pose between t0 and t1 (for example using the tf ROS package).
  This relative incremental pose can then directly be used to get the incremental motion (need to adjust it with the current heading), and to get the covariance of the motion.
*/
namespace perception_oru
{
class MotionModel3d
{
public:
  //! Holds params for the motion models.
  class Params
  {
  public:
    //! Constructor, initiate to reasonable params.
    Params()  {
      set2DParams(0.5, 1.0, 10.0, 10.0, 1.0, 1.0);
      offset<<0.001, 0.001, 0.0, 0.0, 0.00, 0.0001;
    }

    Params(MotionModel2d::Params &params){
      set2DParams(params.Dd,params.Dt, params.Cd, params.Ct, params.Td, params.Tt);
      offset<<0.001, 0.001, 0.0, 0.0, 0.00, 0.0001;
    }

    void set2DParams(double Dd, double Dt, double Cd, double Ct, double Td, double Tt);

    void set3DParams(double Dd, double Dt, double Cd, double Ct, double Bd, double Bt, double Rd, double Rt, double Sd, double St, double Td, double Tt);

    void SetOffset(const Eigen::Matrix<double,6,1> &o){ offset=o; }

    //! Return a one-line condensed string of the parameters
    std::string getDescString() const;

    //! Display the parameters.
    friend std::ostream& operator<<(std::ostream &os, const MotionModel3d::Params &obj){
      os << "- motion model - ";
      os << "\n" << obj.getDescString();
      return os;
    }
    Eigen::Matrix<double, 6, 2> motion_model;
    Eigen::Matrix<double, 6, 1> offset;

  };

  MotionModel3d() { }

  MotionModel3d(const MotionModel3d::Params &p) : params(p) { }

  void setParams(const MotionModel3d::Params &p){params = p;}

  //! Obtain the covariance for the provided relative incremental pose
  Eigen::MatrixXd getCovMatrix(const Eigen::Affine3d &rel) const;

  Eigen::MatrixXd GetCovarianceDiagonal(const Eigen::Affine3d &rel) const;

  MotionModel3d::Params params;

private:

  Eigen::MatrixXd getMeasurementCov(const Eigen::Affine3d &rel) const;


};

} // namespace

#endif
