#include <ndt_generic_lamide/ndt_utils.h>

namespace ndt_generic
{

double L2Distance(const Eigen::Vector3d& mean_a,
                  const Eigen::Matrix3d& cov_a,
                  const Eigen::Vector3d& mean_b,
                  const Eigen::Matrix3d& cov_b)
{
    Eigen::Matrix3d covCombined = cov_a * cov_b;
    Eigen::Matrix3d icov;
    bool exists;
    double det = 0;

    // Get the inverse convariance
    covCombined.computeInverseAndDetWithCheck(icov, det, exists);
    if (!exists)
    {
        return -1;
    }

    // This calculates the likelihood
    double l = (mean_a - mean_b).dot(icov * (mean_a - mean_b));
    if (l * 0 != 0)
    {
        return -1;
    }

    // And this applies the scaling parameters
    // d1 = score_cell_weight + (1.0 - score_cell_weight)
    // d2 => * exp(-d2*l/2.0)
    // d1 = 1.0 traditionally
    double d2 = 0.05;
    double voxelScore = -1 * exp((-d2 / 2.0) * l);

    return voxelScore;
}
} // namespace ndt_generic