#include <ndt_map_lamide/ndt_cell.h>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <ndt_generic_lamide/ndt_utils.h>

namespace perception_oru
{

//  ██████╗ ██████╗ ███╗   ██╗███████╗████████╗███████╗
// ██╔════╝██╔═══██╗████╗  ██║██╔════╝╚══██╔══╝██╔════╝
// ██║     ██║   ██║██╔██╗ ██║███████╗   ██║   ███████╗
// ██║     ██║   ██║██║╚██╗██║╚════██║   ██║   ╚════██║
// ╚██████╗╚██████╔╝██║ ╚████║███████║   ██║   ███████║
//  ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝   ╚═╝   ╚══════╝

bool NDTCell::parametersSet_ = false;
double NDTCell::EVAL_ROUGH_THR = 0;
double NDTCell::EVEC_INCLINED_THR = 0;
double NDTCell::EVAL_FACTOR = 0;
int NDTCell::MIN_NB_POINTS_FOR_GAUSSIAN = 0;
bool NDTCell::CLEAR_MIN_NB_POINTS = false;
bool NDTCell::MIN_NB_POINTS_SET_UNIFORM = false;

//  ██████╗██████╗ ██╗   ██╗██████╗
// ██╔════╝██╔══██╗██║   ██║██╔══██╗
// ██║     ██████╔╝██║   ██║██║  ██║
// ██║     ██╔══██╗██║   ██║██║  ██║
// ╚██████╗██║  ██║╚██████╔╝██████╔╝
//  ╚═════╝╚═╝  ╚═╝ ╚═════╝ ╚═════╝

NDTCell::NDTCell()
{
    InitializeVariables();
}

NDTCell::~NDTCell()
{
    points_.clear();
}

NDTCell::NDTCell(pcl::PointXYZL& center, double& xsize, double& ysize, double& zsize)
{
    InitializeVariables();
    center_ = center;
    xsize_ = xsize;
    ysize_ = ysize;
    zsize_ = zsize;
}

NDTCell::NDTCell(const NDTCell& other)
{
    InitializeVariables();
    this->center_ = other.center_;
    this->xsize_ = other.xsize_;
    this->ysize_ = other.ysize_;
    this->zsize_ = other.zsize_;
    this->hasGaussian_ = other.hasGaussian_;
    this->R = other.R;
    this->G = other.G;
    this->B = other.B;
    this->N = other.N;
    this->setOccupancy(other.getOccupancy());
    this->max_occu_ = other.getMaxOccupancy();
    this->emptyval = other.getEmptyval();
    this->isEmpty = other.isEmpty;
    this->emptydist = other.getEmptyDist();
    this->emptylik = other.getEmptyLik();
    this->setMean(other.getMean());
    this->setCov(other.getCov());
    this->icov_ = other.getInverseCov();
    this->evals_ = other.getEvals();
    this->evecs_ = other.getEvecs();
    this->label_ = other.label_;
    this->label_weight_ = other.label_weight_;
    this->labels_ = other.labels_;
    this->num_labels_ = other.num_labels_;
    this->cluster_id_ = other.cluster_id_;
    this->ray_hit_ = other.ray_hit_;
    this->ray_through_ = other.ray_through_;
    this->comparison_status_ = other.comparison_status_;
    this->comparison_distance_ = other.comparison_distance_;
}

/**
 * produces a new Cell* of the same type
 */
NDTCell* NDTCell::clone() const
{
    NDTCell* ret = new NDTCell();
    ret->setDimensions(this->xsize_, this->ysize_, this->zsize_);
    ret->setCenter(this->center_);
    ret->setRGB(this->R, this->G, this->B);
    ret->setOccupancy(getOccupancy());
    ret->setEmptyval(emptyval);
    ret->setN(this->N);
    ret->isEmpty = this->isEmpty;
    ret->hasGaussian_ = this->hasGaussian_;
    ret->ray_hit_ = this->ray_hit_;
    ret->ray_hit_ = this->ray_through_;
    ret->comparison_status_ = this->comparison_status_;
    ret->comparison_distance_ = this->comparison_distance_;
    return ret;
}

/** produces a new Cell* of the same type and sets it to have the same
 * parameters as this cell.
 */
NDTCell* NDTCell::copy() const
{
    NDTCell* ret = new NDTCell();
    ret->setDimensions(this->xsize_, this->ysize_, this->zsize_);
    ret->setCenter(this->center_);

    for (unsigned int i = 0; i < this->points_.size(); i++)
    {
        ret->points_.push_back(this->points_[i]);
    }

    ret->setMean(this->getMean());
    ret->setCov(this->getCov());
    ret->setRGB(this->R, this->G, this->B);
    ret->setOccupancy(this->getOccupancy());
    ret->setEmptyval(emptyval);
    ret->setN(this->N);
    ret->isEmpty = this->isEmpty;
    ret->hasGaussian_ = this->hasGaussian_;
    ret->ray_hit_ = this->ray_hit_;
    ret->ray_hit_ = this->ray_through_;
    ret->comparison_status_ = this->comparison_status_;
    ret->comparison_distance_ = this->comparison_distance_;
    return ret;
}

void NDTCell::InitializeVariables()
{
    cl_ = UNKNOWN;
    hasGaussian_ = false;
    R = G = B = 0;
    N = 0;
    // Occupancy value stored as "Log odds" (if you wish)
    setOccupancy(0);
    // The number of times a cell was observed empty (using ray casting)
    emptyval = 0;
    isEmpty = 0;
    emptylik = 0;
    emptydist = 0;
    max_occu_ = 100;
    center_.x = center_.y = center_.z = 0;
    xsize_ = ysize_ = zsize_ = 0;
    cov_ = Eigen::MatrixXd::Identity(3, 3);
    icov_ = Eigen::MatrixXd::Identity(3, 3);
    evecs_ = Eigen::MatrixXd::Identity(3, 3);
    mean_ = Eigen::Vector3d(0, 0, 0);
    evals_ = Eigen::Vector3d(0, 0, 0);
    d1_ = d2_ = 0;
    label_ = 0;
    labels_ = Eigen::ArrayXi::Ones(labels::COUNT);
    label_weight_ = 0;
    num_labels_ = 0;
    cluster_id_ = -1;
    ray_hit_ = 0;
    ray_through_ = 0;
    comparison_status_ = NDTCell::ComparisonState::NOT_SET;
    comparison_distance_ = 0;
    if (!parametersSet_)
    {
        setParameters();
    }
}

bool NDTCell::operator==(const NDTCell& b) const
{
    pcl::PointXYZL center = this->center_;
    pcl::PointXYZL other = b.center_;
    return center.x == other.x && center.y == other.y && center.z == other.z;
}

//  ██████╗ ███████╗████████╗        ██╗    ███████╗███████╗████████╗
// ██╔════╝ ██╔════╝╚══██╔══╝       ██╔╝    ██╔════╝██╔════╝╚══██╔══╝
// ██║  ███╗█████╗     ██║         ██╔╝     ███████╗█████╗     ██║
// ██║   ██║██╔══╝     ██║        ██╔╝      ╚════██║██╔══╝     ██║
// ╚██████╔╝███████╗   ██║       ██╔╝       ███████║███████╗   ██║
//  ╚═════╝ ╚══════╝   ╚═╝       ╚═╝        ╚══════╝╚══════╝   ╚═╝

/** setter for covariance
 */
void NDTCell::setCov(const Eigen::Matrix3d& _cov)
{
    cov_ = _cov;
    this->rescaleCovariance();
}

double NDTCell::getDiagonal() const
{
    return std::sqrt(xsize_ * xsize_ + ysize_ * ysize_ + zsize_ * zsize_);
}

// use this to set the parameters for the NDTCell. \note be careful, remember that the
// parameters are static, thus global
void NDTCell::setParameters(double _EVAL_ROUGH_THR,
                            double _EVEC_INCLINED_THR,
                            double _EVAL_FACTOR,
                            int _MIN_NB_POINTS_FOR_GAUSSIAN,
                            bool _CLEAR_MIN_NB_POINTS,
                            bool _MIN_NB_POINTS_SET_UNIFORM)
{

    NDTCell::EVAL_ROUGH_THR = _EVAL_ROUGH_THR;
    NDTCell::EVEC_INCLINED_THR = cos(_EVEC_INCLINED_THR);
    NDTCell::EVAL_FACTOR = _EVAL_FACTOR;
    NDTCell::MIN_NB_POINTS_FOR_GAUSSIAN = _MIN_NB_POINTS_FOR_GAUSSIAN;
    NDTCell::CLEAR_MIN_NB_POINTS = _CLEAR_MIN_NB_POINTS;
    NDTCell::MIN_NB_POINTS_SET_UNIFORM = _MIN_NB_POINTS_SET_UNIFORM;

    parametersSet_ = true;
}

/**
 * Get likelihood for a given point
 */
double NDTCell::getLikelihood(const pcl::PointXYZL& pt) const
{
    // compute likelihood
    if (!hasGaussian_)
    {
        return -1;
    }

    Eigen::Vector3d vec(pt.x, pt.y, pt.z);
    vec = vec - mean_;
    double likelihood = vec.dot(icov_ * vec);

    if (std::isnan(likelihood))
    {
        return -1;
    }

    return exp(-likelihood / 2);
}

void NDTCell::getRGB(float& r, float& g, float& b)
{
    r = R;
    g = G;
    b = B;
}

const double NDTCell::getMaxOccupancy() const
{
    return max_occu_;
}

/**
 * Returns the current accumulated occupancy value
 */
const float NDTCell::getOccupancy() const
{
    return occ;
}

const double NDTCell::getEmptyDist() const
{
    return emptydist;
}

const double NDTCell::getEmptyLik() const
{
    return emptylik;
}

void NDTCell::setOccupancy(const float occ_)
{
    if (!std::isnan(occ_))
    {
        occ = occ_;
    }
}

void NDTCell::setEmptyval(int emptyval_)
{
    emptyval = emptyval_;
}

const int NDTCell::getEmptyval() const
{
    return emptyval;
}

void NDTCell::setN(int N_)
{
    N = N_;
}

int NDTCell::getN()
{
    return N;
}

int NDTCell::getLabel() const
{
    return label_;
}

int NDTCell::getLabelWeight() const
{
    return label_weight_;
}

long NDTCell::getLabelTime() const
{
    return label_time_;
}

void NDTCell::setLabel(int l, int weight)
{
    // No label set or higher weight
    if (label_weight_ == 0 || weight > label_weight_)
    {
        label_ = l;
        label_weight_ = weight;

        time_t timer = time(NULL);
        label_time_ = timer;
    }

    // Set the color
    setRGB(label_);
}

void NDTCell::clearRayStats()
{
    ray_hit_ = 0;
    ray_through_ = 0;
}

void NDTCell::addRayHit()
{
    ray_hit_++;
}

void NDTCell::addRayThrough(double p)
{
    ray_through_p_ += p;
    if(p > 0.5)
    {
        ray_through_++;
    }
}

double NDTCell::getRayHits()
{
    return ray_hit_;
}

double NDTCell::getRayThroughs()
{
    return ray_through_;
}

double NDTCell::getRayThroughP()
{
    return ray_through_p_;
}

double NDTCell::setRayHits(double hits)
{
    ray_hit_ = hits;
}

double NDTCell::setRayThroughs(double throughs)
{
    ray_through_ = throughs;
}

double NDTCell::getRayHitRatio()
{
    double total = ray_hit_ + ray_through_;
    double p = ray_hit_ / total;
    return p;
}

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
void NDTCell::updateSampleVariance(const Eigen::Matrix3d& cov2,
                                   const Eigen::Vector3d& m2,
                                   unsigned int numpointsindistribution,
                                   bool updateOccupancyFlag,
                                   double positiveOccupancy,
                                   float max_occu,
                                   unsigned int maxnumpoints)
{

    if (numpointsindistribution <= 2)
    {
        fprintf(stderr, "updateSampleVariance:: INVALID NUMBER OF POINTS\n");
    }
    if (this->hasGaussian_)
    {
        Eigen::Vector3d msum1 = mean_ * (double)N;
        Eigen::Vector3d msum2 = m2 * (double)numpointsindistribution;

        Eigen::Matrix3d csum1 = cov_ * (double)(N - 1);
        Eigen::Matrix3d csum2 = cov2 * (double)(numpointsindistribution - 1);

        if (fabsf(N) < 1e-5)
        {
            fprintf(stderr, "Divider error (%u %u)!\n", N, numpointsindistribution);
            hasGaussian_ = false;
            return;
        }
        double divider = (double)numpointsindistribution + (double)N;
        if (fabs(divider) < 1e-5)
        {
            fprintf(stderr, "Divider error (%u %u)!\n", N, numpointsindistribution);
            return;
        }
        mean_ = (msum1 + msum2) / (divider);

        double w1 = ((double)N / (double)(numpointsindistribution * (N + numpointsindistribution)));
        double w2 = (double)(numpointsindistribution) / (double)N;
        Eigen::Matrix3d csum3 =
            csum1 + csum2 + w1 * (w2 * msum1 - msum2) * (w2 * msum1 - msum2).transpose();

        // Modifications from Ulf - end
        N = N + numpointsindistribution;
        cov_ = 1.0 / ((double)N - 1.0) * csum3;
        if (updateOccupancyFlag)
        {
            double occval = positiveOccupancy;
            double likoccval = log((occval) / (1.0 - occval));
            float logoddlikoccu = numpointsindistribution * likoccval;
            updateOccupancy(logoddlikoccu, max_occu);
        }
    }
    else
    {
        mean_ = m2;
        cov_ = cov2;
        N = numpointsindistribution;
        hasGaussian_ = true;
        if (updateOccupancyFlag)
        {
            double occval = positiveOccupancy;
            double likoccval = log((occval) / (1.0 - occval));
            float logoddlikoccu = numpointsindistribution * likoccval;
            updateOccupancy(logoddlikoccu, max_occu);
        }
    }
    if (N > maxnumpoints)
    {
        N = maxnumpoints;
    }
    if (this->getOccupancy() < 0)
    {
        this->hasGaussian_ = false;
        return;
    }
    rescaleCovariance();
}

/**
        Attempts to fit a gaussian in the cell.
        computes covariance and mean using observations
*/
void NDTCell::computeGaussian(int mode,
                              unsigned int maxnumpoints,
                              double positiveOccupancy,
                              float occupancy_limit,
                              const Eigen::Vector3d& origin,
                              double sensor_noise)
{
    // Occupancy update part
    // This part infers the given "inconsistency" check done on update phase and
    // updates the values accordingly

    // Continous occupancy update
    double occval = positiveOccupancy;

    double lik = log((occval) / (1.0 - occval));
    double logoddlikoccu = points_.size() * lik;
    // We have to trust that it is occupied if we have measurements from the cell!
    float update = logoddlikoccu;
    updateOccupancy(logoddlikoccu, occupancy_limit);

    isEmpty = -1;
    emptyval = 0;
    emptylik = 0;
    emptydist = 0;

    if (getOccupancy() <= 0)
    {
        hasGaussian_ = false;
        return;
    }

    if (points_.empty())
    {
        return;
    }

    bool clear_points = true;
    bool min_nb_points = false;
    if ((hasGaussian_ == false && points_.size() < MIN_NB_POINTS_FOR_GAUSSIAN))
    {
        if (!MIN_NB_POINTS_SET_UNIFORM && CLEAR_MIN_NB_POINTS)
        {
            points_.clear();
            return;
        }
        if (!MIN_NB_POINTS_SET_UNIFORM)
            return;
        if (!CLEAR_MIN_NB_POINTS)
            clear_points = false;
        min_nb_points = true;
    }

    if (mode == CELL_UPDATE_MODE_STUDENT_T)
    {
        studentT();
        return;
    }

    if (!hasGaussian_)
    {
        computeMeanFromPoints(mean_, points_);
        if (min_nb_points && MIN_NB_POINTS_SET_UNIFORM)
        {
            cov_ = Eigen::MatrixXd::Identity(3, 3);
        }
        else
        {
            computeCovFromPoints(cov_, mean_, points_);
        }
        this->rescaleCovariance();
        N = points_.size();
    }
    // Update using new information
    else if (mode == CELL_UPDATE_MODE_COVARIANCE_INTERSECTION)
    {
        updateCovarianceIntersection();
    }
    else if (mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE)
    {
        updateSampleVariance(maxnumpoints);
    }
    else if (mode == CELL_UPDATE_MODE_ERROR_REFINEMENT)
    {
        updateErrorRefinement();
    }
    else if (mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE_SURFACE_ESTIMATION)
    {
        updateSampleVarianceSurfaceEstimation(maxnumpoints, origin, sensor_noise);
    }
    // End of update
    updateColorInformation();
    if (clear_points)
    {
        points_.clear();
    }
}

void NDTCell::setComparisonStatus(NDTCell::ComparisonState status)
{
    comparison_status_ = status;
}

NDTCell::ComparisonState NDTCell::getComparisonStatus()
{
    return comparison_status_;
}

void NDTCell::setComparisonDistance(double dist)
{
    comparison_distance_ = dist;
}

double NDTCell::getComparisonDistance()
{
    return comparison_distance_;
}

double NDTCell::compare(const NDTCell* other) const
{
    double dist = ndt_generic::L2Distance(this->getMean(), this->getCov(), other->getMean(), other->getCov());
    return dist;
}

// ██╗   ██╗██████╗ ██████╗  █████╗ ████████╗███████╗
// ██║   ██║██╔══██╗██╔══██╗██╔══██╗╚══██╔══╝██╔════╝
// ██║   ██║██████╔╝██║  ██║███████║   ██║   █████╗
// ██║   ██║██╔═══╝ ██║  ██║██╔══██║   ██║   ██╔══╝
// ╚██████╔╝██║     ██████╔╝██║  ██║   ██║   ███████╗
//  ╚═════╝ ╚═╝     ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝

void NDTCell::updateCovarianceIntersection()
{
    Eigen::Vector3d m2;
    m2 << 0, 0, 0;
    for (unsigned int i = 0; i < points_.size(); i++)
    {
        Eigen::Vector3d tmp;
        tmp << points_[i].x, points_[i].y, points_[i].z;
        m2 += tmp;
    }
    m2 /= (points_.size());

    Eigen::MatrixXd mp;
    mp.resize(points_.size(), 3);
    for (unsigned int i = 0; i < points_.size(); i++)
    {
        mp(i, 0) = points_[i].x - m2(0);
        mp(i, 1) = points_[i].y - m2(1);
        mp(i, 2) = points_[i].z - m2(2);
    }
    Eigen::Matrix3d c2;
    c2 = mp.transpose() * mp / (points_.size() - 1);
    double w1 = 0.98;
    Eigen::Matrix3d c3, icov2, icov3;
    bool exists = false;
    double det = 0;
    exists = rescaleCovariance(c2, icov2);

    if (exists)
    {
        c3 = w1 * icov_ + (1.0 - w1) * icov2;
        c3.computeInverseAndDetWithCheck(icov3, det, exists);
        if (exists)
        {
            cov_ = icov3;
            mean_ = icov3 * (w1 * icov_ * mean_ + (1.0 - w1) * icov2 * m2);

            this->rescaleCovariance();
            N += points_.size();
        }
        else
        {
            fprintf(stderr, "Covariance Intersection failed - Inverse does not exist (2)\n");
        }
    }
    else
    {
        points_.clear();
        fprintf(stderr, "Covariance Intersection failed - Inverse does not exist (1)\n");
    }
}

void NDTCell::updateSampleVariance(unsigned int maxnumpoints)
{
    Eigen::Vector3d meanSum_ = mean_ * N;
    Eigen::Matrix3d covSum_ = cov_ * (N - 1);

    Eigen::Vector3d m2;
    m2 << 0, 0, 0;

    for (unsigned int i = 0; i < points_.size(); i++)
    {
        Eigen::Vector3d tmp;
        tmp << points_[i].x, points_[i].y, points_[i].z;
        m2 += tmp;
    }

    Eigen::Vector3d T2 = m2;

    m2 /= (points_.size());

    Eigen::MatrixXd mp;
    mp.resize(points_.size(), 3);
    for (unsigned int i = 0; i < points_.size(); i++)
    {
        mp(i, 0) = points_[i].x - m2(0);
        mp(i, 1) = points_[i].y - m2(1);
        mp(i, 2) = points_[i].z - m2(2);
    }

    Eigen::Matrix3d c2;
    c2 = mp.transpose() * mp;
    Eigen::Matrix3d c3;

    double w1 = ((double)N / (double)(points_.size() * (N + points_.size())));
    double w2 = (double)(points_.size()) / (double)N;

    c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * (w2 * meanSum_ - (T2)).transpose();

    meanSum_ = meanSum_ + T2;
    covSum_ = c3;
    N = N + points_.size();

    /**
     * If the timescaling is set.
     * This does "Sliding Average" for the sample mean and covariance
     */
    if (maxnumpoints > 0)
    {
        if (maxnumpoints < N)
        {
            meanSum_ = meanSum_ * ((double)maxnumpoints / (double)N);
            covSum_ = covSum_ * ((double)(maxnumpoints - 1) / (double)(N - 1));
            N = maxnumpoints;
        }
    }
    mean_ = meanSum_ / (double)N;
    cov_ = covSum_ / (double)(N - 1);
    this->rescaleCovariance();
}

void NDTCell::updateErrorRefinement()
{
    double e_min = 5.0e-4;
    double e_max = 5.0e-2;
    double o_max = 10.0;
    if (getOccupancy() > o_max)
        setOccupancy(o_max);
    if (getOccupancy() < -o_max)
        setOccupancy(-o_max);

    double epsilon = ((e_min - e_max) / (2.0 * o_max)) * (getOccupancy() + o_max) + e_max;

    for (unsigned int i = 0; i < points_.size(); i++)
    {
        Eigen::Vector3d tmp;
        tmp << points_[i].x, points_[i].y, points_[i].z;
        mean_ = mean_ + epsilon * (tmp - mean_);

        cov_ = cov_ + epsilon * ((tmp - mean_) * (tmp - mean_).transpose() - cov_);
    }
    hasGaussian_ = true;
    this->rescaleCovariance();
}

void NDTCell::updateSampleVarianceSurfaceEstimation(unsigned int maxnumpoints,
                                                    const Eigen::Vector3d& origin,
                                                    double sensor_noise)
{
    Eigen::Vector3d meanSum_ = mean_ * N;
    Eigen::Matrix3d covSum_ = cov_ * (N - 1);

    Eigen::Vector3d m2;
    m2 << 0, 0, 0;
    for (unsigned int i = 0; i < points_.size(); i++)
    {
        Eigen::Vector3d tmp;
        tmp << points_[i].x, points_[i].y, points_[i].z;
        m2 += tmp;
    }
    Eigen::Vector3d T2 = m2;

    m2 /= (points_.size());

    pcl::PointXYZL orig;
    orig.x = origin[0];
    orig.y = origin[1];
    orig.z = origin[2];

    Eigen::Vector3d Xm;
    Eigen::Matrix3d c2 = Eigen::Matrix3d::Zero();

    for (unsigned int i = 0; i < points_.size(); i++)
    {
        Eigen::Vector3d X;
        X << points_[i].x, points_[i].y, points_[i].z;

        computeMaximumLikelihoodAlongLine(orig, points_[i], Xm);
        double dist = (origin - X).norm();
        // double sigma_dist = 0.5 * ((dist*dist)/12.0); ///test for distance based sensor noise
        double sigma_dist = 0.5 * ((dist) / 20.0); // test for distance based sensor noise
        double snoise = sigma_dist + sensor_noise;
        double model_trust_given_meas =
            0.49 + 0.5 * exp(-0.5 * ((Xm - X).norm() * (Xm - X).norm()) / (snoise * snoise));

        // double loglik_m = log(model_trust_given_meas / (1-model_trust_given_meas));
        double oval = getOccupancy() / 10.0;
        if (oval > 5.0)
            oval = 5.0;

        double likweight = oval;

        double weight = (1.0 - 1.0 / (1.0 + exp(likweight))) * model_trust_given_meas;
        if (N < 100)
            weight = 0;
        c2 = c2 + (1.0 - weight) * (X - m2) * (X - m2).transpose() +
             weight * (Xm - mean_) * (Xm - mean_).transpose();
    }

    Eigen::Matrix3d c3;

    double w1 = ((double)N / (double)(points_.size() * (N + points_.size())));
    double w2 = (double)(points_.size()) / (double)N;

    c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * (w2 * meanSum_ - (T2)).transpose();

    meanSum_ = meanSum_ + T2;
    covSum_ = c3;
    N = N + points_.size();

    /**
     * If the timescaling is set.
     * This does "Sliding Average" for the sample mean and covariance
     */
    if (maxnumpoints > 0)
    {
        if (maxnumpoints < N)
        {
            meanSum_ = meanSum_ * ((double)maxnumpoints / (double)N);
            covSum_ = covSum_ * ((double)(maxnumpoints - 1) / (double)(N - 1));
            N = maxnumpoints;
        }
    }
    mean_ = meanSum_ / (double)N;
    cov_ = covSum_ / (double)(N - 1);
    this->rescaleCovariance();
}

// Just computes the normal distribution parameters from the points and leaves the points into the
// map
void NDTCell::computeGaussianSimple()
{

    if (points_.size() < MIN_NB_POINTS_FOR_GAUSSIAN)
    {
        if (MIN_NB_POINTS_SET_UNIFORM && points_.size() == MIN_NB_POINTS_FOR_GAUSSIAN - 1)
        {
            computeMeanFromPoints(mean_, points_);
            cov_ = Eigen::MatrixXd::Identity(3, 3);
        }
        if (CLEAR_MIN_NB_POINTS)
        {
            points_.clear();
            return;
        }
        if (!MIN_NB_POINTS_SET_UNIFORM)
        {
            return;
        }
    }
    else
    {
        computeMeanFromPoints(mean_, points_);
        computeCovFromPoints(cov_, mean_, points_);
    }
    this->rescaleCovariance();
    N = points_.size();
    R = 0;
    G = 0;
    B = 0;
}

// Update the color information (template specialization)
// Default - Do nothing
void NDTCell::updateColorInformation()
{
}

/**
 * Rescales the covariance to protect against near sigularities
 * and computes the inverse - This does not change class member values
 */
bool NDTCell::rescaleCovariance(Eigen::Matrix3d& cov, Eigen::Matrix3d& invCov)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol(cov);

    Eigen::Matrix3d evecs;
    Eigen::Vector3d evals;

    evecs = Sol.eigenvectors().real();
    evals = Sol.eigenvalues().real();

    if (evals(0) <= 0 || evals(1) <= 0 || evals(2) <= 0)
    {
        // fprintf(stderr,"Negative Eigenvalues!\n");
        hasGaussian_ = false;
        return false;
    }
    else
    {
        bool recalc = false;
        // guard against near singular matrices::
        int idMax;
        // double minEval = evals.minCoeff(&idMin);
        double maxEval = evals.maxCoeff(&idMax);
        if (maxEval > evals(0) * EVAL_FACTOR)
        {
            evals(0) = evals(idMax) / EVAL_FACTOR;
            recalc = true;
        }
        if (maxEval > evals(1) * EVAL_FACTOR)
        {
            evals(1) = evals(idMax) / EVAL_FACTOR;
            recalc = true;
        }
        if (maxEval > evals(2) * EVAL_FACTOR)
        {
            evals(2) = evals(idMax) / EVAL_FACTOR;
            recalc = true;
        }

        if (recalc)
        {
            Eigen::Matrix3d Lam;
            Lam = evals.asDiagonal();
            cov = evecs * Lam * (evecs.transpose());
        }

        // compute inverse covariance
        Eigen::Matrix3d Lam;
        Lam = evals.asDiagonal();
        invCov = evecs * (Lam.inverse()) * (evecs.transpose());
    }
    return true;
}

void NDTCell::rescaleCovariance()
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol(cov_);

    evecs_ = Sol.eigenvectors().real();
    evals_ = Sol.eigenvalues().real();

    // std::cout<<"evals = [ "<<evals_.transpose()<<"]';\n";
    if (evals_(0) <= 0 || evals_(1) <= 0 || evals_(2) <= 0)
    {
        // fprintf(stderr,"evals <=0\n");
        hasGaussian_ = false;
    }
    else
    {
        hasGaussian_ = true;

        bool recalc = false;
        // guard against near singular matrices::
        int idMax;
        // double minEval = evals.minCoeff(&idMin);
        double maxEval = evals_.maxCoeff(&idMax);
        if (maxEval > evals_(0) * EVAL_FACTOR)
        {
            evals_(0) = evals_(idMax) / EVAL_FACTOR;
            recalc = true;
        }
        if (maxEval > evals_(1) * EVAL_FACTOR)
        {
            evals_(1) = evals_(idMax) / EVAL_FACTOR;
            recalc = true;
        }
        if (maxEval > evals_(2) * EVAL_FACTOR)
        {
            evals_(2) = evals_(idMax) / EVAL_FACTOR;
            recalc = true;
        }

        if (recalc)
        {
            Eigen::Matrix3d Lam;
            Lam = evals_.asDiagonal();
            cov_ = evecs_ * Lam * (evecs_.transpose());
        }
        classify();
        // compute inverse covariance
        Eigen::Matrix3d Lam;
        Lam = (evals_).asDiagonal();
        icov_ = evecs_ * (Lam.inverse()) * (evecs_.transpose());
    }
}

/** classifies the cell according to the covariance matrix properties
    if smallest eigenval is bigger then roughness thershold, it's a rough cell
    evaluate inclination of the corresponding evector and classify as vertica, horizontal or
   inclined
  */
void NDTCell::classify()
{

    cl_ = UNKNOWN;

    Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
    tr = tr.rotate(evecs_);

    int index = -1;
    double minEval = evals_.minCoeff(&index);
    if (index < 0 || index > 2)
        return;

    if (minEval > EVAL_ROUGH_THR)
    {
        cl_ = ROUGH;
    }
    // if 1 eval << other 2 -> planar distr
    else
    {
        // default case -> sloping surface
        cl_ = INCLINED;

        // check the orientation of the vanishing axis
        Eigen::Vector3d e3;
        e3 << 0, 0, 1;

        Eigen::Vector3d minorAxis = evecs_.col(index); // tr*e3;

        // dot product with vertical axis gives us the angle
        double d = minorAxis.dot(e3);
        double l = minorAxis.norm();
        double ac = d / l;
        if (fabsf(ac) < EVEC_INCLINED_THR)
        {
            // angle is nearly perpendicular => vertical surface
            cl_ = VERTICAL;
        }

        if (fabsf(ac) > 1 - EVEC_INCLINED_THR)
        {
            // angle is nearly 0 => horizontal surface
            cl_ = HORIZONTAL;
        }
    }
}

/**
 * Adds a new point to distribution (does not update the distribution)
 * Call computeGaussian() to update the content
 */
void NDTCell::addPoint(const pcl::PointXYZL& pt)
{
    points_.push_back(pt);
}

void NDTCell::addPoints(pcl::PointCloud<pcl::PointXYZL>& pt)
{
    points_.insert(points_.begin(), pt.points.begin(), pt.points.end());
}

void NDTCell::updateEmpty(double elik, double dist)
{
    emptyval++;
    emptylik += elik;
    emptydist += dist;
}

/**
 * Returns the current accumulated occupancy value (rescaled)
 */
float NDTCell::getOccupancyRescaled() const
{
    float occupancy = 1 - 1 / (1 + exp(getOccupancy()));
    return occupancy > 1 ? 1 : (occupancy < 0 ? 0 : occupancy);
}

/**
 * Updates the occupancy value of the cell by summing @occ_val to
 * class variable
 */
void NDTCell::updateOccupancy(float occ_val, float max_occu)
{
    // debug
    if (std::isnan(occ_val))
    {
        std::cout << "***************************************" << std::endl;
        std::cout << "nan update warning" << std::endl;
        std::cout << "***************************************" << std::endl;
        return;
    }
    if (std::isnan(occ))
    {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cout << "nan value warning" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

    float orig = occ;

    occ += occ_val;
    if (occ > max_occu)
    {
        occ = max_occu;
    }
    if (occ < -max_occu)
    {
        occ = -max_occu;
    }
    max_occu_ = max_occu;

    // debug
    viz_occ_update_ = occ - orig;
}

// ███╗   ███╗ █████╗ ██╗  ██╗    ██╗     ██╗  ██╗ ██████╗  ██████╗ ██████╗
// ████╗ ████║██╔══██╗╚██╗██╔╝    ██║     ██║  ██║██╔═══██╗██╔═══██╗██╔══██╗
// ██╔████╔██║███████║ ╚███╔╝     ██║     ███████║██║   ██║██║   ██║██║  ██║
// ██║╚██╔╝██║██╔══██║ ██╔██╗     ██║     ██╔══██║██║   ██║██║   ██║██║  ██║
// ██║ ╚═╝ ██║██║  ██║██╔╝ ██╗    ███████╗██║  ██║╚██████╔╝╚██████╔╝██████╔╝
// ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝    ╚══════╝╚═╝  ╚═╝ ╚═════╝  ╚═════╝ ╚═════╝

/**
 * Computes the maximum likelihood that a point moving along a line
 * defined by two points p1 and p2, gets measured agains the normaldistribution that
 * is within this cell.
 * This is used in raytracing to check if a measurement ray passes through a previously
 * mapped object (thus provides evidence of inconsistency)
 *
 * @param p1 One point along the ray
 * @param p2 second point along the ray (it must hold that p1 != p2);
 */
double NDTCell::computeMaximumLikelihoodAlongLine(const pcl::PointXYZL& p1,
                                                  const pcl::PointXYZL& p2,
                                                  pcl::PointXYZL& out)
{
    Eigen::Vector3d v1, v2;
    v1 << p1.x, p1.y, p1.z;
    v2 << p2.x, p2.y, p2.z;
    Eigen::Vector3d outv;
    double likelihood = computeMaximumLikelihoodAlongLine(v1, v2, outv);
    out.x = outv[0];
    out.y = outv[1];
    out.z = outv[2];
    out.label = p1.label;
    return likelihood;
}

double NDTCell::computeMaximumLikelihoodAlongLine(const pcl::PointXYZL& p1,
                                                  const pcl::PointXYZL& p2,
                                                  Eigen::Vector3d& out)
{
    Eigen::Vector3d v1, v2;
    v1 << p1.x, p1.y, p1.z;
    v2 << p2.x, p2.y, p2.z;
    return computeMaximumLikelihoodAlongLine(v1, v2, out);
}

double NDTCell::computeMaximumLikelihoodAlongLine(const Eigen::Vector3d& v1,
                                                  const Eigen::Vector3d& v2,
                                                  Eigen::Vector3d& out)
{
    Eigen::Vector3d L = (v2 - v1) / (v2 - v1).norm();
    Eigen::Vector3d A = icov_ * L;
    Eigen::Vector3d B = (v2 - mean_);

    double sigma = A(0) * L(0) + A(1) * L(1) + A(2) * L(2);
    if (sigma == 0)
    {
        return 1.0;
    }

    double t = -(A(0) * B(0) + A(1) * B(1) + A(2) * B(2)) / sigma;

    Eigen::Vector3d X = L * t + v2; // X marks the spot

    pcl::PointXYZL p;
    p.x = X(0);
    p.y = X(1);
    p.z = X(2);
    out = X;
    return getLikelihood(p);
}

// ██╗      █████╗ ██████╗ ███████╗██╗     ███████╗
// ██║     ██╔══██╗██╔══██╗██╔════╝██║     ██╔════╝
// ██║     ███████║██████╔╝█████╗  ██║     ███████╗
// ██║     ██╔══██║██╔══██╗██╔══╝  ██║     ╚════██║
// ███████╗██║  ██║██████╔╝███████╗███████╗███████║
// ╚══════╝╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝╚══════╝

int NDTCell::getLabelIndex(const int value)
{
    auto it = find(labels::LABELS.begin(), labels::LABELS.end(), value);

    if (it != labels::LABELS.end())
    {
        return it - labels::LABELS.begin();
    }
    else
    {
        std::cout << "label not found" << std::endl;
        return -1;
    }
}

int NDTCell::getLabelForIndex(const int index)
{
    return labels::LABELS[index];
}

void NDTCell::calculateLabel()
{
    // add label
    for (unsigned int i = 0; i < points_.size(); i++)
    {
        labels_[getLabelIndex(points_[i].label)]++;
    }
    num_labels_ = labels_.sum();

    // find max and set label
    Eigen::Index label_index;
    int weight = labels_.maxCoeff(&label_index);
    setLabel(getLabelForIndex(label_index), weight);

    // calculate distribution
    labels_distribution_ = labels_.cast<double>();
    labels_distribution_ = labels_distribution_ / (double)num_labels_;
}

Eigen::ArrayXd NDTCell::normalizeDistribution(const Eigen::ArrayXi& dist)
{
    Eigen::ArrayXd labels_distribution_ = dist.cast<double>();
    labels_distribution_ = labels_distribution_ / (double)dist.sum();
    return labels_distribution_;
}

const Eigen::ArrayXd NDTCell::getLabelDistribution()
{
    return NDTCell::normalizeDistribution(labels_);
}

const Eigen::ArrayXi& NDTCell::getLabels() const
{
    return labels_;
}

// This should only be called through setLabel
// Set the RGB value for this cell
void NDTCell::setRGB(float r, float g, float b)
{
    R = r;
    G = g;
    B = b;
}

//  ██████╗██╗     ██╗   ██╗███████╗████████╗███████╗██████╗
// ██╔════╝██║     ██║   ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗
// ██║     ██║     ██║   ██║███████╗   ██║   █████╗  ██████╔╝
// ██║     ██║     ██║   ██║╚════██║   ██║   ██╔══╝  ██╔══██╗
// ╚██████╗███████╗╚██████╔╝███████║   ██║   ███████╗██║  ██║
//  ╚═════╝╚══════╝ ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝

void NDTCell::setClusterId(int id, double membership)
{
    cluster_id_ = id;
    cluster_membership_ = membership;
}

void NDTCell::setClusterMembership(double membership)
{
    cluster_membership_ = membership;
}

int NDTCell::getClusterId() const
{
    return cluster_id_;
}

double NDTCell::getClusterMembership() const
{
    return cluster_membership_;
}

//      ██╗███████╗███████╗
//      ██║██╔════╝██╔════╝
//      ██║█████╗  █████╗
// ██   ██║██╔══╝  ██╔══╝
// ╚█████╔╝██║     ██║
//  ╚════╝ ╚═╝     ╚═╝

/** output method to save the ndt cell as part of a jff v0.5 file
 */
int NDTCell::writeToJFF(FILE* jffout)
{
    pcl::PointXYZL* center = &(this->center_);
    fwrite(center, sizeof(pcl::PointXYZL), 1, jffout);
    double cell_size[3] = {this->xsize_, this->ysize_, this->zsize_};
    fwrite(cell_size, sizeof(double), 3, jffout);

    writeJFFMatrix(jffout, cov_);
    writeJFFVector(jffout, mean_);

    // Temporary arrays to write all cell data
    double dtemp[2] = {d1_, d2_};
    // int    itemp[2] = {N, emptyval};
    int itemp[3] = {(int)N, emptyval, (int)hasGaussian_};
    float ftemp[2] = {getOccupancy()}; // float  ftemp[2] ={R, G, B, occ};

    fwrite(dtemp, sizeof(double), 2, jffout);
    fwrite(itemp, sizeof(int), 3, jffout);
    fwrite(ftemp, sizeof(float), 4, jffout);

    // writeJFFEventData(jffout, edata);

    return 0;
}

/** helper function for loadFromJFF()
 */
int NDTCell::loadJFFMatrix(FILE* jffin, Eigen::Matrix3d& mat)
{

    double dtemp[6];

    if (fread(&dtemp, sizeof(double), 6, jffin) <= 0)
        return -1;

    mat(0, 0) = dtemp[0];
    mat(1, 0) = dtemp[1];
    mat(2, 0) = dtemp[2];
    mat(1, 1) = dtemp[3];
    mat(2, 1) = dtemp[4];
    mat(2, 2) = dtemp[5];
    mat(0, 1) = dtemp[1];
    mat(0, 2) = dtemp[2];
    mat(1, 2) = dtemp[4];

    return 0;
}

/** helper function for writeToJFF()
 */
void NDTCell::writeJFFMatrix(FILE* jffout, Eigen::Matrix3d& mat)
{

    double dtemp[6];

    dtemp[0] = mat.coeff(0, 0);
    dtemp[1] = mat.coeff(1, 0);
    dtemp[2] = mat.coeff(2, 0);
    dtemp[3] = mat.coeff(1, 1);
    dtemp[4] = mat.coeff(2, 1);
    dtemp[5] = mat.coeff(2, 2);

    fwrite(dtemp, sizeof(double), 6, jffout);
}

/** another helper function for writeToJFF()
 */
void NDTCell::writeJFFVector(FILE* jffout, Eigen::Vector3d& vec)
{

    double dtemp[3];

    for (int i = 0; i < 3; i++)
    {
        dtemp[i] = vec.coeff(i);
    }

    fwrite(dtemp, sizeof(double), 3, jffout);
}

/** yet another helper function for writeToJFF()
 */
void NDTCell::writeJFFEventData(FILE* jffout, TEventData& evdata)
{
    float ftemp[4];
    uint8_t ocval[1] = {evdata.occval};
    uint64_t evnts[1] = {evdata.events};

    ftemp[0] = evdata.a_exit_event;
    ftemp[1] = evdata.b_exit_event;
    ftemp[2] = evdata.a_entry_event;
    ftemp[3] = evdata.b_entry_event;

    fwrite(ocval, sizeof(uint8_t), 1, jffout);
    fwrite(ftemp, sizeof(float), 4, jffout);
    fwrite(evnts, sizeof(uint64_t), 1, jffout);
}

/** another helper function for loadFromJFF()
 */
int NDTCell::loadJFFVector(FILE* jffin, Eigen::Vector3d& vec)
{
    double dtemp[3];

    if (fread(&dtemp, sizeof(double), 3, jffin) <= 0)
        return -1;

    vec << dtemp[0], dtemp[1], dtemp[2];

    return 0;
}

/** yet another helper function for loadFromJFF()
 */
int NDTCell::loadJFFEventData(FILE* jffin, TEventData& evdata)
{
    float ftemp[4];
    uint8_t ocval;
    uint64_t evnts;

    if (fread(&ocval, sizeof(uint8_t), 1, jffin) <= 0)
    {
        return -1;
    }
    if (fread(&ftemp, sizeof(float), 4, jffin) <= 0)
    {
        return -1;
    }
    if (fread(&evnts, sizeof(uint64_t), 1, jffin) <= 0)
    {
        return -1;
    }

    evdata.a_exit_event = ftemp[0];
    evdata.b_exit_event = ftemp[1];
    evdata.a_entry_event = ftemp[2];
    evdata.b_entry_event = ftemp[3];
    evdata.occval = ocval;
    evdata.events = evnts;

    return 0;
}

/** input method to load the ndt cell from a jff v0.5 file
 */
int NDTCell::loadFromJFF(FILE* jffin)
{

    pcl::PointXYZL center;
    if (fread(&center, sizeof(pcl::PointXYZL), 1, jffin) <= 0)
    {
        return -1;
    }
    this->setCenter(center);

    double dimensions[3];
    if (fread(&dimensions, sizeof(double), 3, jffin) <= 0)
    {
        return -1;
    }
    this->setDimensions(dimensions[0], dimensions[1], dimensions[2]);

    Eigen::Matrix3d temp_matrix;
    Eigen::Vector3d temp_vector;
    if (loadJFFMatrix(jffin, temp_matrix) < 0)
    {
        return -1;
    }
    this->setCov(temp_matrix);

    if (loadJFFVector(jffin, temp_vector) < 0)
    {
        return -1;
    }
    this->setMean(temp_vector);

    // Temporary arrays to load all cell data to
    double dtemp[2]; // = {d1_, d2_};
    int itemp[3];    // = {N, emptyval, hasGaussian_};
    float ftemp[5];  // = {R, G, B, occ, cellConfidence};

    if (fread(&dtemp, sizeof(double), 2, jffin) <= 0)
    {
        return -1;
    }
    if (fread(&itemp, sizeof(int), 3, jffin) <= 0)
    {
        return -1;
    }
    if (fread(&ftemp, sizeof(float), 4, jffin) <= 0)
    {
        return -1;
    }

    this->d1_ = dtemp[0];
    this->d2_ = dtemp[1];
    this->setN(itemp[0]);
    this->setEmptyval(itemp[1]);
    this->hasGaussian_ = (bool)itemp[2];
    this->setOccupancy(ftemp[0]);

    return 0;
}

// ██╗  ██╗███████╗██╗     ██████╗ ███████╗██████╗ ███████╗
// ██║  ██║██╔════╝██║     ██╔══██╗██╔════╝██╔══██╗██╔════╝
// ███████║█████╗  ██║     ██████╔╝█████╗  ██████╔╝███████╗
// ██╔══██║██╔══╝  ██║     ██╔═══╝ ██╔══╝  ██╔══██╗╚════██║
// ██║  ██║███████╗███████╗██║     ███████╗██║  ██║███████║
// ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝

std::string NDTCell::ToString()
{
    std::stringstream ss;
    ss << "\n<<NDTCell: type= " << cl_ << "ParametersSet_=" << parametersSet_
       << "\nHasGaussian_=" << hasGaussian_ << "\nIsEmpty=" << (bool)isEmpty;
    ss << "\nCenter_.x=" << center_.x << ", center_.y=" << center_.y << ", center_.z=" << center_.z
       << "\nXsize_=" << xsize_ << ", ysize_=" << ysize_ << ", zsize_=" << zsize_ << std::endl;
    ss << "mean_=\n" << mean_ << "\ncov_=" << cov_ << ">>" << std::endl;
    return ss.str();
}

// Robust estimation using Student-T
void NDTCell::studentT()
{
    Eigen::Vector3d meanSum_, meantmp_;
    Eigen::Matrix3d covSum_, covTmp_;

    // degrees of freedom of the t-distribution
    double nu = 5;
    // maximum number of iterations
    unsigned int maxIter = 10;
    std::vector<double> lambda;
    unsigned int pnts = points_.size();
    lambda.reserve(pnts);
    for (unsigned int i = 0; i < pnts; i++)
    {
        lambda[i] = 1.0;
    }

    for (unsigned int j = 0; j < maxIter; j++)
    {
        // update mean
        double lambdaSum = 0;
        meantmp_ << 0, 0, 0;
        for (unsigned int i = 0; i < pnts; i++)
        {
            Eigen::Vector3d tmp;
            tmp << points_[i].x, points_[i].y, points_[i].z;
            meantmp_ += lambda[i] * tmp;
            lambdaSum += lambda[i];
        }

        meanSum_ = meantmp_;
        meantmp_ /= lambdaSum;

        // update scalematrix
        Eigen::MatrixXd mp;
        mp.resize(points_.size(), 3);
        for (unsigned int i = 0; i < pnts; i++)
        {
            double sqrtLambda = sqrt(lambda[i]);
            mp(i, 0) = sqrtLambda * (points_[i].x - meantmp_(0));
            mp(i, 1) = sqrtLambda * (points_[i].y - meantmp_(1));
            mp(i, 2) = sqrtLambda * (points_[i].z - meantmp_(2));
        }
        covSum_ = mp.transpose() * mp;
        covTmp_ = covSum_ / (points_.size());

        // compute inverse scalematrix
        Eigen::Matrix3d invCov;
        double det = 0;
        bool exists = false;
        covTmp_.computeInverseAndDetWithCheck(invCov, det, exists);
        if (!exists)
        {
            // The inverse does not exist -- exit
            return;
        }

        Eigen::Vector3d tempVec;
        // update the weights
        for (unsigned int i = 0; i < points_.size(); i++)
        {
            tempVec(0) = points_[i].x - meantmp_(0);
            tempVec(1) = points_[i].y - meantmp_(1);
            tempVec(2) = points_[i].z - meantmp_(2);
            double temp = nu;
            temp += squareSum(invCov, tempVec);
            lambda[i] = (nu + 3) / (temp);
        }
    }
    double temp;
    temp = nu / (nu - 2.0);
    covTmp_ = temp * covTmp_;

    if (!hasGaussian_)
    {
        mean_ = meantmp_;
        cov_ = covTmp_;
        N = pnts;
        this->rescaleCovariance();
    }
    else
    {
        updateSampleVariance(covTmp_, meantmp_, pnts, false);
    }

    points_.clear();
}

double NDTCell::squareSum(const Eigen::Matrix3d& C, const Eigen::Vector3d& x)
{
    double sum;
    sum = C(0, 0) * x(0) * x(0) + C(1, 1) * x(1) * x(1) + C(2, 2) * x(2) * x(2);
    sum += 2.0 * C(0, 1) * x(0) * x(1) + 2.0 * C(0, 2) * x(0) * x(2) + 2.0 * C(1, 2) * x(1) * x(2);
    return sum;
}

void NDTCell::setRGB(int label)
{
    auto it = labels::COLOR_MAP.find(label);
    if (it != labels::COLOR_MAP.end())
    {
        std::vector<int> color = labels::COLOR_MAP.at(label);
        float r = color[0];
        float g = color[1];
        float b = color[2];
        setRGB(r, g, b);
    }
    else
    {
        std::cout << "can't find label " << label << std::endl;
        setRGB(1.0f, 1.0f, 1.0f);
    }
}

}; // namespace perception_oru