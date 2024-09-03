/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef NDT_CELL_HH
#define NDT_CELL_HH

#include <ndt_map_lamide/impl/EventCounterData.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cstdio>
#include <fstream>

#include <Eigen/StdVector>

#include <Eigen/Eigen>
#include "boost/serialization/serialization.hpp"
#include <ndt_map_lamide/labels.h>

/// A rather unsophisticated way of determining the
/// update method for a cell
/// Covariance intersection based on estimation
#define CELL_UPDATE_MODE_COVARIANCE_INTERSECTION 0
/// Recursive Sample variance method [Chan, Gene, Randall, Updating Formulae and pairwise algorithm
/// for computing sample variances, tech report Standford, 1979]
#define CELL_UPDATE_MODE_SAMPLE_VARIANCE 1
/// Yguel, Vasquez, Aycard, Siegward, Laugier, Error-Driven Refinement of Multi-scale gaussian maps
#define CELL_UPDATE_MODE_ERROR_REFINEMENT 2
/// Estimate the surface (reduce the sensor noise)
#define CELL_UPDATE_MODE_SAMPLE_VARIANCE_SURFACE_ESTIMATION 3
/// Student-T
#define CELL_UPDATE_MODE_STUDENT_T 4

#define REFACTORED

#define JFFERR(x)                \
    std::cerr << x << std::endl; \
    return -1;
#define _JFFVERSION_ "#JFF V0.50"

#define KEEP_POINTS

namespace perception_oru
{

/** \brief implements a normal distibution cell
    \details The base class for all NDT indeces, contains
mean, covariance matrix as well as eigen decomposition of covariance
*/

//  ██████╗██╗      █████╗ ███████╗███████╗
// ██╔════╝██║     ██╔══██╗██╔════╝██╔════╝
// ██║     ██║     ███████║███████╗███████╗
// ██║     ██║     ██╔══██║╚════██║╚════██║
// ╚██████╗███████╗██║  ██║███████║███████║
//  ╚═════╝╚══════╝╚═╝  ╚═╝╚══════╝╚══════╝

// template<typename PointT>
class NDTCell //: public Cell<PointT>
{
public:
    enum CellClass
    {
        HORIZONTAL = 0,
        VERTICAL,
        INCLINED,
        ROUGH,
        UNKNOWN
    };

    enum ComparisonState
    {
        NOT_SET = -1,
        OTHER_MISSING = 0,
        THIS_MISSING = 1,
        BOTH_MISSING = 2,
        OTHER_NO_DISTRIBUTION = 3,
        THIS_NO_DISTRIBUTION = 4,
        BOTH_NO_DISTRIBUTION = 5,
        MATCH = 6
    };

    //  ██████╗██████╗ ██╗   ██╗██████╗
    // ██╔════╝██╔══██╗██║   ██║██╔══██╗
    // ██║     ██████╔╝██║   ██║██║  ██║
    // ██║     ██╔══██╗██║   ██║██║  ██║
    // ╚██████╗██║  ██║╚██████╔╝██████╔╝
    //  ╚═════╝╚═╝  ╚═╝ ╚═════╝ ╚═════╝
    NDTCell();

    virtual ~NDTCell();

    NDTCell(pcl::PointXYZL& center, double& xsize, double& ysize, double& zsize);

    NDTCell(const NDTCell& other);

    virtual NDTCell* clone() const;

    virtual NDTCell* copy() const;

    void InitializeVariables();

    bool operator==(const NDTCell& b) const;

    // ██╗███╗   ██╗██╗     ██╗███╗   ██╗███████╗
    // ██║████╗  ██║██║     ██║████╗  ██║██╔════╝
    // ██║██╔██╗ ██║██║     ██║██╔██╗ ██║█████╗
    // ██║██║╚██╗██║██║     ██║██║╚██╗██║██╔══╝
    // ██║██║ ╚████║███████╗██║██║ ╚████║███████╗
    // ╚═╝╚═╝  ╚═══╝╚══════╝╚═╝╚═╝  ╚═══╝╚══════╝

    inline void setCenter(const pcl::PointXYZL& cn)
    {
        center_ = cn;
    }

    inline void setDimensions(const double& xs, const double& ys, const double& zs)
    {
        xsize_ = xs;
        ysize_ = ys;
        zsize_ = zs;
    }

    inline pcl::PointXYZL getCenter() const
    {
        return center_;
    }

    inline void getCenter(double& cx, double& cy, double& cz) const
    {
        cx = center_.x;
        cy = center_.y;
        cz = center_.z;
    }

    inline void getDimensions(double& xs, double& ys, double& zs) const
    {
        xs = xsize_;
        ys = ysize_;
        zs = zsize_;
    }

    inline bool isInside(const pcl::PointXYZL pt) const
    {
        if (pt.x < center_.x - xsize_ / 2 || pt.x > center_.x + xsize_ / 2)
        {
            return false;
        }
        if (pt.y < center_.y - ysize_ / 2 || pt.y > center_.y + ysize_ / 2)
        {
            return false;
        }
        if (pt.z < center_.z - zsize_ / 2 || pt.z > center_.z + zsize_ / 2)
        {
            return false;
        }
        return true;
    }

    inline CellClass getClass() const
    {
        return cl_;
    }
    inline Eigen::Matrix3d getCov() const
    {
        return cov_;
    }
    inline Eigen::Matrix3d getInverseCov() const
    {
        return icov_;
    }
    inline Eigen::Vector3d getMean() const
    {
        return mean_;
    }
    inline Eigen::Matrix3d getEvecs() const
    {
        return evecs_;
    }
    inline Eigen::Vector3d getEvals() const
    {
        return evals_;
    }

    inline void setMean(const Eigen::Vector3d& mean)
    {
        mean_ = mean;
    }
    inline void setEvals(const Eigen::Vector3d& ev)
    {
        evals_ = ev;
    }

    /// FOR TESTING ONLY
    inline void setEvecs(const Eigen::Matrix3d& ev)
    {
        evecs_ = ev;
    }

    inline void computeMeanFromPoints(
        Eigen::Vector3d& mean_,
        const std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>>& points_)
    {

        Eigen::Vector3d meanSum_;
        mean_ << 0, 0, 0;
        for (unsigned int i = 0; i < points_.size(); i++)
        {
            Eigen::Vector3d tmp;
            tmp << points_[i].x, points_[i].y, points_[i].z;
            mean_ += tmp;
        }
        meanSum_ = mean_;
        mean_ /= (points_.size());
    }

    inline void computeCovFromPoints(
        Eigen::Matrix3d& cov_,
        const Eigen::Vector3d& mean_,
        const std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>>& points_)
    {

        Eigen::MatrixXd mp;

        mp.resize(points_.size(), 3);
        for (unsigned int i = 0; i < points_.size(); i++)
        {
            mp(i, 0) = points_[i].x - mean_(0);
            mp(i, 1) = points_[i].y - mean_(1);
            mp(i, 2) = points_[i].z - mean_(2);
        }
        Eigen::Matrix3d covSum_ = mp.transpose() * mp;
        cov_ = covSum_ / (points_.size() - 1);
    }

    //  ██████╗ ███████╗████████╗        ██╗    ███████╗███████╗████████╗
    // ██╔════╝ ██╔════╝╚══██╔══╝       ██╔╝    ██╔════╝██╔════╝╚══██╔══╝
    // ██║  ███╗█████╗     ██║         ██╔╝     ███████╗█████╗     ██║
    // ██║   ██║██╔══╝     ██║        ██╔╝      ╚════██║██╔══╝     ██║
    // ╚██████╔╝███████╗   ██║       ██╔╝       ███████║███████╗   ██║
    //  ╚═════╝ ╚══════╝   ╚═╝       ╚═╝        ╚══════╝╚══════╝   ╚═╝

    /// use this to set the parameters for the NDTCell. \note be careful, remember that the
    /// parameters are static, thus global
    static void setParameters(double _EVAL_ROUGH_THR = 0.1,
                              double _EVEC_INCLINED_THR = 8 * M_PI / 18,
                              double _EVAL_FACTOR = 1000,
                              int _MIN_NB_POINTS_FOR_GAUSSIAN = 3,
                              bool _CLEAR_MIN_NB_POINTS = true,
                              bool _MIN_NB_POINTS_SET_UNIFORM = false);

    virtual double getDiagonal() const;

    void setCov(const Eigen::Matrix3d& cov);

    /**
     * Get likelihood for a given point
     */
    double getLikelihood(const pcl::PointXYZL& pt) const;

    void getRGB(float& r, float& g, float& b);

    const double getMaxOccupancy() const;

    /**
     * Returns the current accumulated occupancy value
     */
    const float getOccupancy() const;

    const double getEmptyDist() const;

    const double getEmptyLik() const;

    void setOccupancy(const float occ_);

    void setEmptyval(int emptyval_);

    const int getEmptyval() const;

    void setN(int N_);

    int getN();

    int getLabel() const;

    int getLabelWeight() const;

    long getLabelTime() const;

    void setLabel(int l, int weight);

    void clearRayStats();

    void addRayHit();

    void addRayThrough(double p);

    double getRayHits();

    double getRayThroughs();

    double getRayThroughP();

    double setRayHits(double hits);

    double setRayThroughs(double throughs);

    double getRayHitRatio();

    void setComparisonStatus(NDTCell::ComparisonState status);

    NDTCell::ComparisonState getComparisonStatus();

    void setComparisonDistance(double dist);

    double getComparisonDistance();

    // ███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗
    // ██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║
    // █████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║
    // ██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║
    // ██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║
    // ╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝

    /**
     * Updates the current Sample mean and covariance based on
     * give new sample mean @m2 and covariance @cov2,
     * which have been computed from @numpointsindistribution number of points
     */

    void updateSampleVariance(const Eigen::Matrix3d& cov2,
                              const Eigen::Vector3d& m2,
                              unsigned int numpointsindistribution,
                              bool updateOccupancyFlag = true,
                              double positiveOccupancy = 0.6,
                              float max_occu = 1024,
                              unsigned int maxnumpoints = 1e9);

    /**
     * Fits and updates the sample mean and covariance for the cell after the scan has been added.
     * This function updates the occupancy, it uses the given method for the update.
     * The behavior of the update can be altered with axnumpoints and occupancy_limit
     *
     * @param mode Determines the mode of the cell update - the mode defines are in the beginning of
     * the header
     * @param maxnumpoints This adapts the cell content in the presence of non-stationary
     * distribution. The lower the value the faster the adaptation
     * @param occupancy_limit This sets the limit for the confidence (in log-odds) that the cell can
     * obtain.
     * @param origin The CELL_UPDATE_MODE_SAMPLE_VARIANCE_SURFACE_ESTIMATION requires to know the
     * sensor origin, so in case You use that, you should provide the position of the sensor, from
     * where the measurement was taken for the method
     * @param sensor_noise A standard deviation of the sensor noise, used only by method
     * CELL_UPDATE_MODE_SAMPLE_VARIANCE_SURFACE_ESTIMATION if (maxnumpoints<=0) then the cell
     * adaptation strategy is not used
     */
    void computeGaussian(int mode = CELL_UPDATE_MODE_SAMPLE_VARIANCE,
                         unsigned int maxnumpoints = 1e9,
                         double positiveOccupancy = 0.6,
                         float occupancy_limit = 255,
                         const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                         double sensor_noise = 0.1);

    void updateCovarianceIntersection();
    void updateSampleVariance(unsigned int maxnumpoints);
    void updateErrorRefinement();
    void updateSampleVarianceSurfaceEstimation(unsigned int maxnumpoints,
                                               const Eigen::Vector3d& origin,
                                               double sensor_noise);

    /**
     * just updates the parameters based on points and leaves the points to cell
     */
    void computeGaussianSimple();
    /**
     * Calculates the average color for cell if the point type is pcl::PointXYZLI or
     * pcl::PointXYZRGBL
     */
    void updateColorInformation();

    void rescaleCovariance();
    /**
     * Rescales the covariance to protect against near sigularities
     * and computes the inverse - This does not change class member values
     * @return true if success, false if eigen values were negative
     */
    bool rescaleCovariance(Eigen::Matrix3d& cov, Eigen::Matrix3d& invCov);

    void classify();

    int writeToJFF(FILE* jffout);

    int loadFromJFF(FILE* jffin);

    /**
     * Adds a new point to distribution (does not update the distribution)
     * Call computeGaussian() to update the content
     */
    virtual void addPoint(const pcl::PointXYZL& pt);

    virtual void addPoints(pcl::PointCloud<pcl::PointXYZL>& pt);

    void updateEmpty(double elik, double dist);

    /**
     * Returns the current accumulated occupancy value (rescaled)
     */
    float getOccupancyRescaled() const;

    /**
     * Updates the occupancy value of the cell by summing @occ_val to
     * class variable
     */
    void updateOccupancy(float occ_val, float max_occu = 255.0);

    /**
     * Computes the maximum likelihood that a point moving along a line
     * defined by two points p1 and p2, gets measured agains the normaldistribution that
     * is within this cell.
     * This is used in raytracing to check if a measurement ray passes through a previously
     * mapped object (thus provides evidence of inconsistency)
     *
     * @param p1 One point along the ray
     * @param p2 second point along the ray (it must hold that p1 != p2);
     * @param &out Gives out the exact maximum likelihood point
     */
    double computeMaximumLikelihoodAlongLine(const pcl::PointXYZL& p1,
                                             const pcl::PointXYZL& p2,
                                             pcl::PointXYZL& out);

    double computeMaximumLikelihoodAlongLine(const pcl::PointXYZL& p1,
                                             const pcl::PointXYZL& p2,
                                             Eigen::Vector3d& out);

    double computeMaximumLikelihoodAlongLine(const Eigen::Vector3d& ep1,
                                             const Eigen::Vector3d& ep2,
                                             Eigen::Vector3d& out);

    std::string ToString();

    double compare(const NDTCell* other) const;

    // ██╗      █████╗ ██████╗ ███████╗██╗     ███████╗
    // ██║     ██╔══██╗██╔══██╗██╔════╝██║     ██╔════╝
    // ██║     ███████║██████╔╝█████╗  ██║     ███████╗
    // ██║     ██╔══██║██╔══██╗██╔══╝  ██║     ╚════██║
    // ███████╗██║  ██║██████╔╝███████╗███████╗███████║
    // ╚══════╝╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝╚══════╝

    int getLabelIndex(const int value);

    int getLabelForIndex(const int index);

    void calculateLabel();

    static Eigen::ArrayXd normalizeDistribution(const Eigen::ArrayXi& dist);

    const Eigen::ArrayXd getLabelDistribution();

    const Eigen::ArrayXi& getLabels() const;

    // This should only be called through setLabel
    /// Set the RGB value for this cell
    void setRGB(float r, float g, float b);

    //  ██████╗██╗     ██╗   ██╗███████╗████████╗███████╗██████╗
    // ██╔════╝██║     ██║   ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗
    // ██║     ██║     ██║   ██║███████╗   ██║   █████╗  ██████╔╝
    // ██║     ██║     ██║   ██║╚════██║   ██║   ██╔══╝  ██╔══██╗
    // ╚██████╗███████╗╚██████╔╝███████║   ██║   ███████╗██║  ██║
    //  ╚═════╝╚══════╝ ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝

    void setClusterId(int id, double membership = 1.0);
    void setClusterMembership(double membership);
    int getClusterId() const;
    double getClusterMembership() const;

    // ██████╗ ██╗   ██╗██████╗ ██╗     ██╗ ██████╗    ██╗   ██╗ █████╗ ██████╗ ███████╗
    // ██╔══██╗██║   ██║██╔══██╗██║     ██║██╔════╝    ██║   ██║██╔══██╗██╔══██╗██╔════╝
    // ██████╔╝██║   ██║██████╔╝██║     ██║██║         ██║   ██║███████║██████╔╝███████╗
    // ██╔═══╝ ██║   ██║██╔══██╗██║     ██║██║         ╚██╗ ██╔╝██╔══██║██╔══██╗╚════██║
    // ██║     ╚██████╔╝██████╔╝███████╗██║╚██████╗     ╚████╔╝ ██║  ██║██║  ██║███████║
    // ╚═╝      ╚═════╝ ╚═════╝ ╚══════╝╚═╝ ╚═════╝      ╚═══╝  ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝

public:
    bool hasGaussian_; ///< indicates if the cell has a gaussian in it
    char isEmpty; ///< based on the most recent observation, is the cell seen empty (1), occupied
                  ///< (-1) or not at all (0)
                  // double consistency_score;

    // note debugging

    bool viz_updated_ = false;
    bool viz_accessed_ = false;
    bool viz_created_ = false;
    float viz_occ_update_ = 0;

    std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>>
        points_; /// The points falling into the cell - cleared after update

    // ██████╗ ██████╗ ██╗██╗   ██╗ █████╗ ████████╗███████╗
    // ██╔══██╗██╔══██╗██║██║   ██║██╔══██╗╚══██╔══╝██╔════╝
    // ██████╔╝██████╔╝██║██║   ██║███████║   ██║   █████╗
    // ██╔═══╝ ██╔══██╗██║╚██╗ ██╔╝██╔══██║   ██║   ██╔══╝
    // ██║     ██║  ██║██║ ╚████╔╝ ██║  ██║   ██║   ███████╗
    // ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚═╝  ╚═╝   ╚═╝   ╚══════╝

    // ██╗   ██╗ █████╗ ██████╗ ███████╗
    // ██║   ██║██╔══██╗██╔══██╗██╔════╝
    // ██║   ██║███████║██████╔╝███████╗
    // ╚██╗ ██╔╝██╔══██║██╔══██╗╚════██║
    //  ╚████╔╝ ██║  ██║██║  ██║███████║
    //   ╚═══╝  ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝

private:
    pcl::PointXYZL center_;
    double xsize_, ysize_, zsize_;
    Eigen::Matrix3d cov_; /// Contains the covariance of the normal distribution
    /// Precomputed inverse covariance (updated every time the cell is updated)
    Eigen::Matrix3d icov_;
    Eigen::Matrix3d evecs_; /// Eigen vectors
    Eigen::Vector3d mean_;  /// Mean of the normal distribution
    Eigen::Vector3d evals_; /// Eigen values
    CellClass cl_;
    static bool parametersSet_;      // ???
    static double EVAL_ROUGH_THR;    // = 0.1;								// ???
    static double EVEC_INCLINED_THR; // = cos(8*M_PI/18);//10 degree slope;	// ???
    static double EVAL_FACTOR;       // ???
    static int MIN_NB_POINTS_FOR_GAUSSIAN;
    static bool CLEAR_MIN_NB_POINTS;
    static bool MIN_NB_POINTS_SET_UNIFORM;
    double d1_, d2_;
    unsigned int N; /// Number of points used for Normal distribution estimation so far
    int emptyval;   /// The number of times a cell was observed empty (using ray casting)
    double emptylik;
    double emptydist;
    float R, G, B; /// RGB values [0..1] - Special implementations for PointXYZRGB & PointXYZI
    float occ;     /// Occupancy value stored as "Log odds" (if you wish)
    float max_occu_;

    double ray_through_p_;
    double ray_through_;
    double ray_hit_;

    NDTCell::ComparisonState comparison_status_;
    double comparison_distance_;

    // Note: Semantic label
    Eigen::ArrayXi labels_;
    Eigen::ArrayXd labels_distribution_;
    int num_labels_;
    int label_;
    int label_weight_;
    long label_time_;

    // Note: cluster id
    int cluster_id_;
    double cluster_membership_;

    // ███╗   ███╗███████╗███╗   ███╗██████╗ ███████╗██████╗ ███████╗
    // ████╗ ████║██╔════╝████╗ ████║██╔══██╗██╔════╝██╔══██╗██╔════╝
    // ██╔████╔██║█████╗  ██╔████╔██║██████╔╝█████╗  ██████╔╝███████╗
    // ██║╚██╔╝██║██╔══╝  ██║╚██╔╝██║██╔══██╗██╔══╝  ██╔══██╗╚════██║
    // ██║ ╚═╝ ██║███████╗██║ ╚═╝ ██║██████╔╝███████╗██║  ██║███████║
    // ╚═╝     ╚═╝╚══════╝╚═╝     ╚═╝╚═════╝ ╚══════╝╚═╝  ╚═╝╚══════╝

    void setRGB(int label);

    /**
     * Cell estimation using student-t
     */
    void studentT();

    double squareSum(const Eigen::Matrix3d& C, const Eigen::Vector3d& x);

    void writeJFFMatrix(FILE* jffout, Eigen::Matrix3d& mat);
    void writeJFFVector(FILE* jffout, Eigen::Vector3d& vec);
    void writeJFFEventData(FILE* jffout, TEventData& evdata);
    int loadJFFMatrix(FILE* jffin, Eigen::Matrix3d& mat);
    int loadJFFVector(FILE* jffin, Eigen::Vector3d& vec);
    int loadJFFEventData(FILE* jffin, TEventData& evdata);

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(
        Archive& ar,
        const unsigned int version) // In order to clal this you need to register it to boost
                                    // using "ar.template register_type<LazyGrid>();"
    {
        ar& hasGaussian_;
        // based on the most recent observation, is the cell seen empty (1),
        // occupied
        // (-1) or not at all (0)
        ar& isEmpty;
        ar& center_;
        ar& xsize_& ysize_& zsize_;
        ar& cov_;
        ar& icov_;  /// Precomputed inverse covariance (updated every time the cell is updated)
        ar& evecs_; /// Eigen vectors
        ar& boost::serialization::make_array(mean_.data(), 3);
        ar& boost::serialization::make_array(evals_.data(), 3);
        ar& parametersSet_;
        ar& EVAL_ROUGH_THR;
        ar& EVEC_INCLINED_THR;
        ar& EVAL_FACTOR;
        ar& d1_& d2_; // remove?
        ar& N;
        ar& emptyval;
        ar& emptylik;
        ar& emptydist;

        ar& R& G& B;
        ar& label_;
        ar& label_weight_;
        ar& label_time_;
        ar& boost::serialization::make_array(labels_.data(), labels_.size());
        ar& num_labels_;
        ar& cluster_id_;
        ar& cluster_membership_;
        // ORU_FIXME: this would be better done with a new cell type
        ar& occ;
        ar& max_occu_;
        ar& cl_;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}; // namespace perception_oru

#endif
