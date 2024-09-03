#include <ndt_generic_lamide/math_utils.h>
#include <algorithm>

// Kullback-Leibler divergence
double KLDivergence(const Eigen::ArrayXd& P, const Eigen::ArrayXd& Q)
{
    if(P.rows() == Q.rows() && P.cols() == Q.cols())
    {
        return (P * (P / Q).log()).sum();
    }
    else
    {
        return 10;
    }
}

// Jensen-Shannon divergence
double JSDivergence(const Eigen::ArrayXd& P, const Eigen::ArrayXd& Q)
{
    Eigen::ArrayXd M = 0.5 * (P + Q);
    return 0.5 * (KLDivergence(P, M) + KLDivergence(Q, M));
}

double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

double weightingFunction(double divergence)
{
    return 1.0 - clamp(std::exp(-divergence), 0.0, 1.0);
}