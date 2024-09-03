#include "ndt_generic_lamide/motion_model_3d.h"
namespace perception_oru
{

std::string MotionModel3d::Params::getDescString() const
{
    std::ostringstream os;
    os << motion_model;
    return os.str();
}

void MotionModel3d::Params::set2DParams(
    double Dd, double Dt, double Cd, double Ct, double Td, double Tt)
{
    // Variance forward from distance
    // Variance forward fom rotation
    // the abouve pairs for x (Dd, Dt), y (Cd, Ct) , z, roll, pitch, yaw (Td, Tt)
    motion_model(0, 0) = Dd;
    motion_model(0, 1) = Dt;

    motion_model(1, 0) = Cd;
    motion_model(1, 1) = Ct;

    motion_model(4, 1) = motion_model(4, 0) = motion_model(3, 1) = motion_model(3, 0) =
        motion_model(2, 1) = motion_model(2, 0) = 0.0;

    motion_model(5, 0) = Td;
    motion_model(5, 1) = Tt;
}

void MotionModel3d::Params::set3DParams(double Dd,
                                        double Dt,
                                        double Cd,
                                        double Ct,
                                        double Bd,
                                        double Bt,
                                        double Rd,
                                        double Rt,
                                        double Sd,
                                        double St,
                                        double Td,
                                        double Tt)
{
    set2DParams(Dd, Dt, Cd, Ct, Td, Tt);

    motion_model(2, 0) = Bd; // Set the rest of the 3d parameters not set by the above call
    motion_model(2, 1) = Bt;
    motion_model(3, 0) = Rd;
    motion_model(3, 1) = Rt;
    motion_model(4, 0) = Sd;
    motion_model(4, 1) = St;
}

Eigen::MatrixXd MotionModel3d::GetCovarianceDiagonal(const Eigen::Affine3d& rel) const
{
    double dist = rel.translation().norm();
    Eigen::Vector3d euler = rel.rotation().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(euler);
    double rot = euler.norm();

    Eigen::Matrix<double, 2, 1> incr;
    incr(0, 0) = dist * dist;
    incr(1, 0) = rot * rot;
    auto ret = params.motion_model * incr;

    return ret;
}

Eigen::MatrixXd MotionModel3d::getMeasurementCov(const Eigen::Affine3d& rel) const
{
    Eigen::MatrixXd diag = GetCovarianceDiagonal(rel);
    Eigen::MatrixXd R(6, 6);
    R.setZero();
    for (int i = 0; i < 6; i++)
        R(i, i) = diag(i);

    return R;
}

Eigen::MatrixXd MotionModel3d::getCovMatrix(const Eigen::Affine3d& rel) const
{
    return getMeasurementCov(rel);
}

} // namespace perception_oru
