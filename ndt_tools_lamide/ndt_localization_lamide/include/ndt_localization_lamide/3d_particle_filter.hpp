/**
 * This is an implementation for 3D (or 6D to be more precise) particle filter.
 * @author Jari Saarinen (jari.p.saarinen@gmail.com)
 */

#ifndef _Particle_filter_3D_h_
#define _Particle_filter_3D_h_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <Eigen/Core>
#include <random>
#include "ndt_localization_lamide/particle.hpp"
#include <stdio.h>
#include <iostream>

// #include "ndt_mcl_lamide/ownRandom.h"
#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"
#include "angles/angles.h"
using std::cout;
using std::endl;
namespace perception_oru
{

typedef std::vector<particle, Eigen::aligned_allocator<particle>> Particles;
struct normal_random_variable
{
    normal_random_variable(Eigen::Matrix3d& covar)
        : normal_random_variable(
              Eigen::Vector3d::Zero(covar.rows()), covar, Eigen::Vector3d::Zero(covar.rows()))
    {
    }

    normal_random_variable()
        : mean(Eigen::Vector3d::Zero())
        , transform(Eigen::Matrix3d::Identity())
        , scaling(Eigen::Vector3d::Zero())
    {
    }

    normal_random_variable(Eigen::Vector3d const& mean,
                           Eigen::Matrix3d const& covar,
                           const Eigen::Vector3d& scaling)
        : mean(mean)
        , scaling(scaling)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covar);
        Eigen::Matrix3d eig_val = eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
        eig_val = scaling.asDiagonal() * eig_val;
        transform = eigenSolver.eigenvectors() * eig_val;
    }
    void SetTransform(Eigen::Vector3d const& u,
                      Eigen::Matrix3d const& covar,
                      const Eigen::Vector3d& scale)
    {
        mean = u;
        scaling = scale;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covar);
        Eigen::Matrix3d eig_val = eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
        eig_val = eig_val * scaling.asDiagonal();
        transform = eigenSolver.eigenvectors() * eig_val;
    }

    Eigen::Vector3d mean;
    Eigen::Vector3d scaling;
    Eigen::Matrix3d transform;
    Eigen::Vector3d GetSample() const
    {
        static std::mt19937 gen{std::random_device{}()};
        static std::normal_distribution<> dist{0, 1};
        Eigen::Vector3d v(mean.size());
        for (int i = 0; i < mean.size(); i++)
            v[i] = dist(gen);
        return mean + transform * v;
        // return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return
        // dist(gen); });
    }
};

class particle_filter_3d
{
public:
    Particles pcloud; ///< Particle distribution
    // ownRandom myrand;                 			///< The random number class

    particle_filter_3d()
    {
    }

    /**
     * Initializes the filter by sampling from normal distribution with
     * mean in @p0 and variance defined by @variance
     */
    void initializeNormalRandom(unsigned int NumParticles,
                                double mx,
                                double my,
                                double mz,
                                double mroll,
                                double mpitch,
                                double myaw,
                                double vx,
                                double vy,
                                double vz,
                                double vroll,
                                double vpitch,
                                double vyaw,
                                bool keep_current_cloud = false);

    void initializeNormalRandom(unsigned int NumParticles,
                                double mx,
                                double my,
                                double mz,
                                double mroll,
                                double mpitch,
                                double myaw,
                                double vx,
                                double vy,
                                double vz,
                                double vroll,
                                double vpitch,
                                double vyaw,
                                double init_prob);

    void initializeUniformRandom(unsigned int NumParticles,
                                double mx,
                                double my,
                                double mz,
                                double mroll,
                                double mpitch,
                                double myaw,
                                double dx,
                                double dy,
                                double dz,
                                double droll,
                                double dpitch,
                                double dyaw,
                                bool keep_current_cloud = false);

    double getEffectiveParticles();
    double getEffectiveParticleRatio();

    void initializeFromVector(
        std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& particles);

    static void normalizeEulerAngles(Eigen::Vector3d& euler);

    /**
     * SIR Update for the filter
     * nr_particles = 0 mantains the current size. Otherwise, <nr_particles> of points are sampled
     * from the particles.
     */
    void SIRUpdate(const unsigned int nr_particles = 0);

    unsigned int size() const
    {
        return pcloud.size();
    }

    //    void initializeByCovarianceMatrix(unsigned int NumParticles, Eigen::Affine3d &Tmean,
    //    Eigen::MatrixXd &cov, const Eigen::Matrix<double,6,1> &scaling);

    static bool SortByProbability(const particle& lhs, const particle& rhs)
    {
        return lhs.probability > rhs.probability;
    }
    /**
     * Performs the normalization step
     * i.e. according to updated likelihoods the probability of each
     * particle is calculated and the whole distribution sums up to 1
     */
    void normalize();

    void predict(Eigen::Affine3d Tmotion,
                 double vx,
                 double vy,
                 double vz,
                 double vroll,
                 double vpitch,
                 double vyaw,
                 const Eigen::Affine3d offset = Eigen::Affine3d::Identity());

    Eigen::Affine3d getMean() const;

    Eigen::Affine3d getMeanFiltered(
        float percent_inliers = 0.5,
        int max = 6000); // if more particles than this exsits, it will not be used

    Eigen::Matrix3d getCovariance() const;

    void Merge(perception_oru::particle_filter_3d& pf, bool delete_input = false);

    bool AverageProbability(double& avg);

    bool MaxProbability(double& max);

    /**
     * Helper to convert xyzrpy to eigen affine3d
     */
    inline Eigen::Affine3d xyzrpy2affine(
        double x, double y, double z, double roll, double pitch, double yaw)
    {
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Translation3d v(x, y, z);
        return (v * m);
    }

    void printStatistics();
    //void printParticle(particle particle);
};

} // namespace perception_oru

#endif
