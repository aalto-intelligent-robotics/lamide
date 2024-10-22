#include <graph_localization_lamide/ukf_ndt/3d_ndt_ukf.h>


Eigen::VectorXd UKF3D::computePoseMean() const
{
    Eigen::VectorXd ret(N_);
    ret.setZero();

    std::vector<double> weights = getMeanWeights();
    std::vector<Eigen::Affine3d> T_sigmas = getSigmasAsAffine3d();

    Eigen::Affine3d T_mean = ndt_generic::getAffine3dMeanWeightsUsingQuatNaive(T_sigmas, weights);

    ret = ndt_generic::affine3dToVector(T_mean);

    ndt_generic::normalizeEulerAngles6dVec(ret);
    return ret;
}

Eigen::MatrixXd UKF3D::computePoseCov(const Eigen::VectorXd& mean) const
{
    Eigen::MatrixXd ret(N_, N_);
    ret.setZero();
    assert(mean.size() == N_);

    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {

        Eigen::VectorXd diff = mean - this->getSigma(i);

        // Normalize the diff angles...
        diff(3) = angles::normalize_angle(diff(3)); // daniel changed to 3 4 5 from 4 5 6
        diff(4) = angles::normalize_angle(diff(4));
        diff(5) = angles::normalize_angle(diff(5));

        ret += getWeightCov(i) * diff * diff.transpose();

        if (fabs(diff(3)) > M_PI / 2.)
        { // changed to 3
            std::cout << "diff way to large" << std::endl;
            std::cout << "diff : " << diff << std::endl;
        }
        if (fabs(diff(4)) > M_PI / 2.)
        { // changed to 4
            std::cout << "diff way to large" << std::endl;
            std::cout << "diff : " << diff << std::endl;
        }
        if (fabs(diff(5)) > M_PI / 2.)
        {
            std::cout << "diff way to large" << std::endl;
            std::cout << "diff : " << diff << std::endl;
        }
    }

    // This don't work since the x,y,z will be dependent on the angles.
    // std::vector<Eigen::Affine3d> T_sigmas = this->getSigmasAsAffine3d();
    // Eigen::Affine3d T_mean = ndt_generic::vectorToAffine3d(mean);

    // for (int i = 0; i < T_sigmas.size(); i++) {
    //     Eigen::Affine3d T_diff = T_sigmas[i]*T_mean.inverse();
    //     Eigen::VectorXd diff = ndt_generic::affine3dToVector(T_diff);

    //     // diff(4) = angles::normalize_angle(diff(4));
    //     // diff(5) = angles::normalize_angle(diff(5));
    //     // diff(6) = angles::normalize_angle(diff(6));

    //     ndt_generic::normalizeEulerAngles6dVec(diff);

    //     std::cout << "diff [" << i << "] : " << diff.transpose() << std::endl;

    //     ret += getWeightCov(i) * diff * diff.transpose();
    // }

    return ret;
}

std::vector<double> UKF3D::getMeanWeights() const
{
    std::vector<double> ret;

    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {
        ret.push_back(getWeightMean(i));
    }
    return ret;
}

std::vector<double> UKF3D::getCovWeights() const
{
    std::vector<double> ret;

    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {
        ret.push_back(getWeightCov(i));
    }
    return ret;
}

double UKF3D::getWeightMean(int idx) const
{
    const double lambda = this->getLambda();
    if (idx == 0)
    {
        return (lambda / (N_ + lambda));
    }
    return 1. / (2 * (N_ + lambda));
}

double UKF3D::getWeightCov(int idx) const
{
    const double lambda = this->getLambda();
    if (idx == 0)
    {
        return lambda / (N_ + lambda) + (1 - params_.alpha * params_.alpha + params_.beta);
    }
    return 1. / (2 * (N_ + lambda));
}

std::vector<Eigen::Affine3d> UKF3D::getSigmasAsAffine3d() const
{
    std::vector<Eigen::Affine3d> ret;
    if (sigmas_.empty())
    {
        return ret;
    }

    assert(sigmas_.size() == 2 * N_ + 1);

    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {
        ret.push_back(ndt_generic::vectorToAffine3d(sigmas_[i]));
    }

    return ret;
}

void UKF3D::assignSigmas(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{

    // Sigmas is a 6d vector with x,y,z,r,p,y
    sigmas_[0] = mean;

    // Compute the cholesky decomposition (chol(cov)' * chol(cov) = cov)
    Eigen::MatrixXd L(cov.llt().matrixL());

    int idx = 1;
    double xsi = getXsi();
    for (int i = 0; i < N_; i++)
    {
        sigmas_[idx] = mean - xsi * L.col(i);
        idx++;
        sigmas_[idx] = mean + xsi * L.col(i);
        idx++;
    }

    P_ = cov;
}

void UKF3D::predict(const Eigen::Affine3d& incr, const Eigen::MatrixXd& incrCov)
{

    // Take the current estimate with cov and compute the sigma points.

    Eigen::VectorXd mean = ndt_generic::affine3dToVector(T_);
    // Assign the new sigma points
    assignSigmas(mean, P_);

    // Add the incremental pose to all sigmas (noise could be added here).
    for (int i = 0; i < sigmas_.size(); i++)
    {
        Eigen::Affine3d s = ndt_generic::vectorToAffine3d(sigmas_[i]) * incr;
        sigmas_[i] = ndt_generic::affine3dToVector(s);
    }

    // Compute the new mean and cov
    if (params_.debug)
    {
        std::cout << "T_ old [pred] : " << ndt_generic::affine3dToStringRotMat(T_) << std::endl;
    }
    mean = this->computePoseMean();
    T_ = ndt_generic::vectorToAffine3d(mean);
    if (params_.debug)
    {
        std::cout << "T_ new [pred] : " << ndt_generic::affine3dToStringRotMat(T_) << std::endl;
        std::cout << "P_ old [pred] : " << P_ << std::endl;
    }
    Eigen::MatrixXd P = this->computePoseCov(mean);
    P_ = P + incrCov;
    if (params_.debug)
    {
        std::cout << "P_ new [pred] : " << P_ << std::endl;
    }

    assignSigmas(mean, P_);
}

void UKF3D::update(const Eigen::MatrixXd& pred_ranges, const Eigen::VectorXd& raw_ranges)
{

    // Compute the mean of the ray traced measurements
    Eigen::VectorXd mean(raw_ranges.size());
    mean.setZero();

    double weight_m0 = this->getWeightMean(0);
    double weight_m = this->getWeightMean(1);

    for (int i = 0; i < raw_ranges.size(); i++)
    {
        mean(i) = weight_m0 * pred_ranges(0, i);
        for (int j = 1; j < this->getNbSigmaPoints(); j++)
        {
            mean(i) += weight_m * pred_ranges(j, i);
        }
    }

    // Compute the covariance of the ray traced masurements
    Eigen::MatrixXd Q(raw_ranges.size(), raw_ranges.size());
    Q.setIdentity();
    Q *= params_.range_var; // Range variance.

    Eigen::MatrixXd S(raw_ranges.size(), raw_ranges.size());
    S.setZero();

    // Keep the range differences
    std::vector<Eigen::VectorXd> diffs;

    double weight_c0 = this->getWeightCov(0);
    double weight_c = this->getWeightCov(1);

    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {
        Eigen::VectorXd diff = pred_ranges.row(i).transpose() - raw_ranges;
        if (i == 0)
        {
            S += weight_c0 * diff * diff.transpose();
        }
        else
        {
            S += weight_c * diff * diff.transpose();
        }
        diffs.push_back(diff);
    }
    S += Q;

    // Correct the pose
    Eigen::MatrixXd J_t(raw_ranges.size(), 6);
    J_t.setZero();

    // Need to have all the states in a vector format.
    // Current pose, this is already predicted.
    Eigen::VectorXd pred_s = ndt_generic::affine3dToVector(this->T_);

    // Compute the difference between the predicted state and the sigma points
    std::vector<Eigen::VectorXd> s_diffs;
    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {
        s_diffs.push_back(this->sigmas_[i] - pred_s);
        // This need to be specifically checked to avoid 2*M_PI overflows.
        ndt_generic::normalizeEulerAngles6dVec(s_diffs.back());
    }

    for (int i = 0; i < this->getNbSigmaPoints(); i++)
    {
        J_t += this->getWeightCov(i) * diffs[i] * (s_diffs[i].transpose());
    }

    // The inversion is very costly...
    Eigen::MatrixXd K_t = J_t.transpose() * S.colPivHouseholderQr().solve(
                                                Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    // Update the state
    Eigen::VectorXd new_state = pred_s + K_t * (raw_ranges - mean);

    T_ = ndt_generic::vectorToAffine3d(new_state);

    if (params_.debug)
    {
        std::cout << "K_t*(raw_ranges - mean) : " << K_t * (raw_ranges - mean) << std::endl;
        std::cout << "T_ old [update] : " << pred_s << std::endl;
        std::cout << "T_ new [update] : " << new_state << std::endl;
        std::cout << "K_t * S * K_t.transpose() : " << K_t * S * K_t.transpose() << std::endl;
        std::cout << "P_ old [update] : " << P_ << std::endl;
    }
    P_ = P_ - K_t * S * K_t.transpose();
    if (params_.debug)
    {
        std::cout << "P_ new [update] : " << P_ << std::endl;
    }
    assignSigmas(new_state, P_);
}

void UKF3D::update(const Eigen::Affine3d& pose, const Eigen::MatrixXd& cov)
{

    // Update the filter with a global pose along with a covariance
    Eigen::VectorXd measured_s = ndt_generic::affine3dToVector(pose);
    Eigen::VectorXd pred_s = ndt_generic::affine3dToVector(this->T_);

    Eigen::VectorXd diff_s = measured_s - pred_s;
    ndt_generic::normalizeEulerAngles6dVec(diff_s);

    Eigen::MatrixXd Q = cov;

    Eigen::MatrixXd S =
        P_ + Q; // The 'H matrix' is the identiy matrix, we're simply measuring the state(!)
    Eigen::MatrixXd K_t =
        P_ * S.colPivHouseholderQr().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    // Update the state
    Eigen::VectorXd new_state = pred_s + K_t * (diff_s);

    if (params_.debug)
    {
        std::cout << "T_ old [update] : " << pred_s << std::endl;
        std::cout << "T_ new [update] : " << new_state << std::endl;
        std::cout << "P_ old [update] : " << P_ << std::endl;
    }
    T_ = ndt_generic::vectorToAffine3d(new_state);
    P_ = P_ - K_t * S * K_t.transpose();
    if (params_.debug)
    {
        std::cout << "P_ new [update] : " << P_ << std::endl;
    }
}

void UKF3D::updateSeq(const Eigen::MatrixXd& pred_ranges, const Eigen::VectorXd& raw_ranges)
{

    // Split the upate into multiple updates.
    int i = 0;
    int counter = 0;
    while (i < raw_ranges.size() - 1 && counter < params_.nb_updates)
    {

        int nb_ranges = params_.nb_ranges_in_update;
        if (i + nb_ranges >= raw_ranges.size())
        {
            nb_ranges = raw_ranges.size() - i - 1;
        }

        this->update(pred_ranges.block(0, i, getNbSigmaPoints(), nb_ranges),
                     raw_ranges.segment(i, nb_ranges));

        i += params_.nb_ranges_in_update;
        counter++;
    }
}

void UKF3D::assignMinDiagVariances()
{
    double min_var;
    for (int i = 0; i < N_; i++)
    {
        if (i < 3)
            min_var = params_.min_pos_var;
        else
            min_var = params_.min_rot_var;

        if (P_(i, i) < min_var)
        {
            P_(i, i) = min_var;
        }
    }
}

std::vector<int> UKF3D::filterMeasurements(Eigen::MatrixXd& pred_ranges,
                                           Eigen::VectorXd& raw_ranges,
                                           Eigen::MatrixXd& filter_pred_ranges,
                                           Eigen::VectorXd& filter_raw_ranges) const
{

    // Check the difference between the measured ones and the predicted.
    // If the difference is > max_filter then skip this measurement
    std::vector<int> idx;
    for (int i = 0; i < raw_ranges.size(); i++)
    {
        bool use = true;
        for (int j = 0; j < this->getNbSigmaPoints(); j++)
        {
            double diff = fabs(pred_ranges(j, i) - raw_ranges(i));
            if (diff > params_.range_filter_max_dist)
            {
                use = false;
            }
        }
        if (use)
        {
            idx.push_back(i);
        }
    }

    filter_pred_ranges.resize(this->getNbSigmaPoints(), idx.size());
    filter_raw_ranges.resize(idx.size());
    for (int i = 0; i < idx.size(); i++)
    {
        filter_raw_ranges[i] = raw_ranges[idx[i]];
        filter_pred_ranges.col(i) = pred_ranges.col(idx[i]); // Row is per sigma point
    }

    return idx;
}

//-------------

void NDTUKF3D::predict(Eigen::Affine3d Tmotion, const Eigen::MatrixXd& incrCov)
{

    ukf_.predict(Tmotion, incrCov);
}

// Only to visualize the filtered points, the center sigma point pose will be used.
void NDTUKF3D::updateVisualizationClouds(const std::vector<int>& idx,
                                         const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                         const Eigen::MatrixXd& filter_pred_ranges,
                                         const Eigen::Affine3d& Tsensor)
{

    pc_filtered_raw_.clear();
    pc_filtered_pred_.clear();

    for (int i = 0; i < idx.size(); i++)
    {
        pc_filtered_raw_.push_back(cloud[idx[i]]);
        // Only have the distance for the pred...
        Eigen::Vector3d p =
            Tsensor.translation() + dirs_in_vehicle_frame_[idx[i]] * filter_pred_ranges(0, i);
        // std::cout << "|" <<  filter_pred_ranges(0,i) << std::flush;
        pc_filtered_pred_.push_back(ndt_generic::eigenToPCLPoint(p));
    }
}

void NDTUKF3D::update(pcl::PointCloud<pcl::PointXYZL>& cloud, const Eigen::Affine3d& Tsensor)
{

    Eigen::MatrixXd pred_ranges, filter_pred_ranges;
    Eigen::VectorXd raw_ranges, filter_raw_ranges;

    this->computeMeasurements(cloud, Tsensor, pred_ranges, raw_ranges);
    std::vector<int> filter_idx =
        ukf_.filterMeasurements(pred_ranges, raw_ranges, filter_pred_ranges, filter_raw_ranges);

    updateVisualizationClouds(filter_idx, cloud, filter_pred_ranges, Tsensor);

    // ukf_.update(pred_ranges, raw_ranges);
    // ukf_.update(filter_pred_ranges, filter_raw_ranges);
    ukf_.updateSeq(filter_pred_ranges, filter_raw_ranges);

    ukf_.assignMinDiagVariances();
}

void NDTUKF3D::update(const Eigen::Affine3d& pose, const Eigen::MatrixXd& cov)
{
    ukf_.update(pose, cov);
    ukf_.assignMinDiagVariances();
}

// The cloud should be given in the vehicle's frame
void NDTUKF3D::computeMeasurements(pcl::PointCloud<pcl::PointXYZL>& cloud,
                                   const Eigen::Affine3d& Tsensor,
                                   Eigen::MatrixXd& measurements,
                                   Eigen::VectorXd& raw_ranges)
{
    std::vector<Eigen::Vector3d> dirs;
    std::vector<double> r;
    ndt_generic::computeDirectionsAndRangesFromPointCloud(
        cloud, Tsensor.translation(), dirs,
        r); // Vehicle frame (if the sensor is rotated this will directly be encoded in the dirs).

    raw_ranges = Eigen::VectorXd::Map(r.data(), r.size());

    // Cloud is given in the vehicle frame. Hence the cloud should / cloud be tranformed around
    // using the sigma points. One option is instead of rotating the points around simply update the
    // directional vectors (the translation is instead only added to the pose to be computed).

    std::vector<Eigen::Affine3d> sigmas =
        ukf_.getSigmasAsAffine3d(); // These are in vehicle frames...

    measurements.resize(sigmas.size(), raw_ranges.size());

    dirs_in_vehicle_frame_ = dirs;
    std::vector<Eigen::Vector3d> d(dirs.size());

    for (int i = 0; i < sigmas.size(); i++)
    {
        Eigen::Matrix3d rot = (map_offset_ * sigmas[i]).linear();

        for (int j = 0; j < d.size(); j++)
        {
            d[j] = rot * dirs[j];
        }

        Eigen::Vector3d origin = (map_offset_ * sigmas[i] * Tsensor).translation();

        //        std::cout << "origin : " << origin<< std::endl;
        //        std::cout << "map_offset _ : " << map_offset_.translation() << std::endl;
        //        std::cout << "Tsensor : " << Tsensor.translation() << std::endl;
        //        std::cout << "sigmas : " << sigmas[i].translation() << std::endl;

        Eigen::VectorXd ranges;
        if (map_ == NULL)
        {
            std::cout << "NO MAP IS SET" << std::endl;
            exit(-1);
        }
        map_->computeMaximumLikelihoodRanges(origin, raw_ranges, d, ranges);
        measurements.row(i) = ranges;
    }
}

void NDTUKF3D::setMapOffset(const Eigen::Affine3d& offset)
{
    map_offset_ = offset;
}
