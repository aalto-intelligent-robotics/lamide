#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Dense>

// Kullback-Leibler divergence
double KLDivergence(const Eigen::ArrayXd& P, const Eigen::ArrayXd& Q);

// Jensen-Shannon divergence
double JSDivergence(const Eigen::ArrayXd& P, const Eigen::ArrayXd& Q);

// This is used to calculate the coefficient from the divergence
double weightingFunction(double divergence);

double clamp(double n, double lower, double upper);

#endif