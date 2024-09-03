#include <ndt_generic_lamide/eigen_utils.h>

namespace ndt_generic
{

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Affine3dSTLVek;

double GetDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    return (p2 - p1).norm();
}

double getCondition(const Eigen::MatrixXd& m)
{

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m);
    return svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
}

void normalizeEulerAngles(Eigen::Vector3d& euler)
{
    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);

    if (fabs(euler[0]) > M_PI / 2)
    {
        euler[0] += M_PI;
        euler[1] = -euler[1] + M_PI;
        euler[2] += M_PI;

        euler[0] = angles::normalize_angle(euler[0]);
        euler[1] = angles::normalize_angle(euler[1]);
        euler[2] = angles::normalize_angle(euler[2]);
    }
}
double GetDistance(const Eigen::Affine3d& p1, const Eigen::Affine3d& p2)
{
    Eigen::Affine3d diff = p1.inverse() * p2;
    return diff.translation().norm();
}
void normalizeEulerAngles6dVec(Eigen::VectorXd& v)
{

    v[3] = angles::normalize_angle(v[3]);
    v[4] = angles::normalize_angle(v[4]);
    v[5] = angles::normalize_angle(v[5]);

    if (fabs(v[3]) > M_PI / 2.)
    {
        v[3] += M_PI;
        v[4] = -v[4] + M_PI;
        v[5] += M_PI;

        v[3] = angles::normalize_angle(v[3]);
        v[4] = angles::normalize_angle(v[4]);
        v[5] = angles::normalize_angle(v[5]);
    }
}

double SearchForClosestElement(const Eigen::Affine3d& pose,
                               const Affine3dSTLVek& affine3d_vek,
                               Eigen::Affine3d& closest_pose,
                               bool& node_found)
{
    double min_distance = DBL_MAX;
    int index_of_element = -1;
    closest_pose = Eigen::Affine3d::Identity();
    node_found = false;
    for (int i = 0; i < affine3d_vek.size(); i++)
    {
        double distance = GetDistance(pose, affine3d_vek[i]);
        if (distance < min_distance)
        {
            closest_pose = affine3d_vek[i];
            min_distance = distance;
            index_of_element = i;
            node_found = true;
        }
    }
    return min_distance;
}

double SearchForClosestElement(const Eigen::Affine3d& pose, const Affine3dSTLVek& affine3d_vek)
{
    Eigen::Affine3d closest_pose;
    bool node_found;
    return SearchForClosestElement(pose, affine3d_vek, closest_pose, node_found);
}
std::string transformToEvalString(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T)
{
    std::ostringstream stream;
    // stream << std::setprecision(std::numeric_limits<double>::digits10);
    stream << std::setprecision(6);
    Eigen::Quaternion<double> tmp(T.rotation());
    stream << T.translation()(0) << " " << T.translation()(1) << " " << T.translation()(2) << " "
           << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w();
    return stream.str();
}

std::string transformToEvalStringNoLineEnd(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T)
{
    std::ostringstream stream;
    stream << std::setprecision(6);
    Eigen::Quaternion<double> tmp(T.rotation());
    stream << " " << T.translation()(0) << " " << T.translation()(1) << " " << T.translation()(2)
           << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w();
    return stream.str();
    // String.format("%.4f", T.translation()(0), T.translation()(1), T.translation()(2), tmp.x(),
    // tmp.y(), tmp.z(), tmp.z());
}

std::string affine3dToStringRPY(const Eigen::Affine3d& T)
{
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::digits10);
    Eigen::Vector3d rot = T.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(rot);

    stream << T.translation().transpose() << " " << rot.transpose();
    return stream.str();
}

std::string affine3dToStringRotMat(const Eigen::Affine3d& T)
{
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::digits10);

    stream << "transl:\n" << T.translation().transpose() << "\nrot:\n" << T.rotation();
    return stream.str();
}

void typeCastStdVectorToEigen(std::vector<double>& v, Eigen::VectorXd& ev)
{
    ev = Eigen::VectorXd::Map(v.data(), v.size());
}

Eigen::Affine3d getAffine3dMean(const std::vector<Eigen::Affine3d>& Ts)
{

    Eigen::Vector3d sumTransl(0., 0., 0.);
    Eigen::Matrix3d sumRot = Eigen::Matrix3d::Zero();

    const double factor = 1. / (Ts.size() * 1.);

    // This works nice to average the rotation. However if there are different
    // weights (e.g. the factor is not constant this is *NOT* a suitable
    // approach(!)
    for (unsigned int i = 0; i < Ts.size(); i++)
    {
        sumTransl += factor * Ts[i].translation();
        sumRot += factor * Ts[i].rotation();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sumRot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d averageRot = svd.matrixU() * svd.matrixV().transpose();

    Eigen::Affine3d ret;

    ret.translation() = sumTransl;
    ret.linear() = averageRot;
    return ret;
}

// In case the weights have large variance this is likely to fail - use
// getAffine3dMeanWeightsUsingQuat() instead.
Eigen::Affine3d getAffine3dMeanWeights(const std::vector<Eigen::Affine3d>& Ts,
                                       const std::vector<double>& weights)
{

    Eigen::Vector3d sumTransl(0., 0., 0.);
    Eigen::Matrix3d sumRot = Eigen::Matrix3d::Zero();

    double sum_weights = 0;

    for (unsigned int i = 0; i < Ts.size(); i++)
    {

        sumTransl += weights[i] * Ts[i].translation();
        sumRot += weights[i] * Ts[i].rotation();
        sum_weights += weights[i];
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sumRot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d averageRot = svd.matrixU() * svd.matrixV().transpose();

    Eigen::Affine3d ret;
    ret.translation() = sumTransl;
    ret.linear() = averageRot;
    return ret;
}

// ORU_BUG - check this, has to be a bug here...
Eigen::Affine3d getAffine3dMeanWeightsUsingQuat(const std::vector<Eigen::Affine3d>& Ts,
                                                const std::vector<double>& weights)
{

    Eigen::Vector3d sumTransl(0., 0., 0.);
    Eigen::Matrix3d sumRot = Eigen::Matrix3d::Zero();

    double sum_weights = 0;

    Eigen::MatrixXd Q(4, Ts.size());

    for (unsigned int i = 0; i < Ts.size(); i++)
    {

        sumTransl += weights[i] * Ts[i].translation();

        Eigen::Quaterniond q(Ts[i].rotation());
        Q.col(i) = weights[i] * q.coeffs();
    }

    Eigen::Matrix4d Q2 = Q * Q.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(Q2);
    if (eigensolver.info() != Eigen::Success)
        abort(); // this is positive semi-definite - should always work.

    // The eigenvector with the largest eigenvalue is what we want.
    // According the eigen they are not sorted in any way... simply check the for
    // the largest eigen vector.
    int idx;
    eigensolver.eigenvalues().maxCoeff(&idx);

    Eigen::Vector4d ev = eigensolver.eigenvectors().row(idx);
    ev.normalize();

    Eigen::Affine3d ret;
    ret.translation() = sumTransl;
    Eigen::Quaterniond q(ev(3), ev(0), ev(1), ev(2));
    ret.linear() = q.toRotationMatrix();

    return ret;
}

Eigen::Affine3d getAffine3dMeanWeightsUsingQuatNaive(const std::vector<Eigen::Affine3d>& Ts,
                                                     const std::vector<double>& weights)
{

    Eigen::Vector3d sumTransl(0., 0., 0.);
    Eigen::Matrix3d sumRot = Eigen::Matrix3d::Zero();

    double sum_weights = 0;

    Eigen::MatrixXd Q(4, Ts.size());
    Eigen::Vector4d v;
    v.setZero();

    for (unsigned int i = 0; i < Ts.size(); i++)
    {

        sumTransl += weights[i] * Ts[i].translation();

        Eigen::Quaterniond q(Ts[i].rotation());
        v += weights[i] * q.coeffs();
        sum_weights += weights[i];
    }
    v.normalize();

    Eigen::Affine3d ret;
    ret.translation() = sumTransl;

    Eigen::Quaterniond q(v(3), v(0), v(1), v(2));
    ret.linear() = q.toRotationMatrix();

    return ret;
}

Eigen::Affine3d vectorToAffine3d(const Eigen::VectorXd& v)
{

    return Eigen::Translation<double, 3>(v(0), v(1), v(2)) *
           Eigen::AngleAxis<double>(v(3), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxis<double>(v(4), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxis<double>(v(5), Eigen::Vector3d::UnitZ());
}
Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double>& v)
{
    Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
    Eigen::Affine3d T;
    T.linear() = q.toRotationMatrix();
    T.translation() << v[0], v[1], v[2];
    return T;
}

Eigen::Affine3d vectorsToAffine3d(const Eigen::Vector3d& transl, const Eigen::Vector3d& rot)
{
    Eigen::VectorXd v(6);
    v(0) = transl(0);
    v(1) = transl(1);
    v(2) = transl(2);
    v(3) = rot(0);
    v(4) = rot(1);
    v(5) = rot(2);

    return vectorToAffine3d(v);
}
Eigen::Affine3d xyzrpyToAffine3d(double x, double y, double z, double r, double p, double t)
{
    return Eigen::Translation<double, 3>(x, y, z) *
           Eigen::AngleAxis<double>(r, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxis<double>(p, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxis<double>(t, Eigen::Vector3d::UnitZ());
}
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q)
{
    Eigen::Matrix3d m = q.matrix();
    return m.eulerAngles(0, 1, 2);
}

Eigen::VectorXd affine3dToVector(const Eigen::Affine3d& T)
{
    Eigen::VectorXd ret(6);

    Eigen::Vector3d transl = T.translation();
    ret(0) = transl(0);
    ret(1) = transl(1);
    ret(2) = transl(2);

    Eigen::Vector3d rot = T.rotation().eulerAngles(0, 1, 2);
    ret(3) = rot(0);
    ret(4) = rot(1);
    ret(5) = rot(2);

    normalizeEulerAngles6dVec(ret);

    return ret;
}

void affine3dToVectors(const Eigen::Affine3d& T, Eigen::Vector3d& transl, Eigen::Vector3d& rot)
{
    Eigen::VectorXd v = affine3dToVector(T);
    transl(0) = v(0);
    transl(1) = v(1);
    transl(2) = v(2);
    rot(0) = v(3);
    rot(1) = v(4);
    rot(2) = v(5);

    normalizeEulerAngles(rot);
}

Eigen::Vector3d getWeightedPoint(const Eigen::Vector3d& pt1,
                                 const Eigen::Matrix3d& cov1,
                                 const Eigen::Vector3d& pt2,
                                 const Eigen::MatrixXd& cov2)
{
    Eigen::Matrix3d cov1_inv = cov1.inverse();
    Eigen::Matrix3d cov2_inv = cov2.inverse();

    return (cov1_inv + cov2_inv).inverse() * (cov1_inv * pt1 + cov2_inv * pt2);
}

Eigen::Affine3d getWeightedPose(const Eigen::Affine3d& pose1,
                                const Eigen::MatrixXd& cov1,
                                const Eigen::Affine3d& pose2,
                                const Eigen::MatrixXd& cov2)
{
    Eigen::VectorXd p1 = ndt_generic::affine3dToVector(pose1);
    ndt_generic::normalizeEulerAngles6dVec(p1);
    Eigen::VectorXd p2 = ndt_generic::affine3dToVector(pose2);
    ndt_generic::normalizeEulerAngles6dVec(p2);

    std::cout << "p1 : " << p1.transpose() << std::endl;
    std::cout << "p2 : " << p2.transpose() << std::endl;

    Eigen::MatrixXd cov1_inv = cov1.inverse();
    Eigen::MatrixXd cov2_inv = cov2.inverse();

    std::cout << "cov1_inv : " << cov1_inv << std::endl;
    std::cout << "cov2_inv : " << cov2_inv << std::endl;

    Eigen::VectorXd out = (cov1_inv + cov2_inv).inverse() * (cov1_inv * p1 + cov2_inv * p2);

    return vectorToAffine3d(out);
}

double getYaw(const Eigen::Affine3d& T)
{
    Eigen::Vector3d rot = T.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(rot);
    return rot(2);
}

void updateAffineRotationFromEuler(Eigen::Affine3d& T, Eigen::Vector3d& euler)
{
    T.linear() = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()))
                     .toRotationMatrix();
}

void updateRollPitch(Eigen::Affine3d& T, Eigen::Vector3d& euler)
{

    double yaw = getYaw(T);
    T.linear() = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                     .toRotationMatrix();
}
/*!
 * \brief Extrapolate
 * \param T1
 * \param T2
 * \param stepsize
 * \return
 */
Eigen::Affine3d Extrapolate(const Eigen::Affine3d& T1, const Eigen::Affine3d& T2, double stepsize)
{
    Eigen::Affine3d result;
    result = T2 * (T1.inverse() * T2) *
             Eigen::Scaling(stepsize); // factor=1 corresponds to an step equal to a frame
}
Eigen::Affine3d Interpolate(const Eigen::Affine3d& T1, const Eigen::Affine3d& T2, double factor)
{
    // std::cout<<"factor="<<factor<<std::endl;
    Eigen::Affine3d pose;
    Eigen::Quaterniond q1(T1.linear());
    Eigen::Quaterniond q2(T2.linear());
    pose.linear() = (q1.slerp(factor, q2)).toRotationMatrix();
    Eigen::Vector3d tdiff = T2.translation() - T1.translation();
    pose.translation() = T1.translation() + tdiff * factor;
    return pose;
}
Eigen::Affine3f Interpolate(const Eigen::Affine3f& T1, const Eigen::Affine3f& T2, double factor)
{
    // std::cout<<"factor="<<factor<<std::endl;
    Eigen::Affine3f pose;
    Eigen::Quaternionf q1(T1.linear());
    Eigen::Quaternionf q2(T2.linear());
    pose.linear() = (q1.slerp(factor, q2)).toRotationMatrix();
    Eigen::Vector3f tdiff = T2.translation() - T1.translation();
    pose.translation() = T1.translation() + tdiff * factor;
    return pose;
}

bool AlmostEqual(double a, double b)
{
    return fabs(a - b) < std::numeric_limits<double>::epsilon();
}
Eigen::Matrix3d Cov6dTo3d(const Eigen::Matrix<double, 6, 6>& Cov)
{
    if (Cov.cols() != 6 || Cov.rows() != 6)
    {
        std::cerr << "Invalid size of covariance matrix for 6d to 2d conversion" << std::endl;
        exit(0);
    }
    Eigen::Matrix3d cov_2d;
    cov_2d << Cov(0, 0), Cov(0, 1), Cov(0, 5), Cov(1, 0), Cov(1, 1), Cov(1, 5), Cov(5, 0),
        Cov(5, 1), Cov(5, 5);
    return cov_2d;
}
Eigen::Affine3d MuxAffineComponents(const Eigen::Affine3d& T1,
                                    const Eigen::Affine3d& T2,
                                    bool t1x,
                                    bool t1y,
                                    bool t1z,
                                    bool t1ex,
                                    bool t1ey,
                                    bool t1ez)
{

    Eigen::Vector3d t1rot = T1.rotation().eulerAngles(0, 1, 2);
    Eigen::Vector3d t2rot = T2.rotation().eulerAngles(0, 1, 2);
    normalizeEulerAngles(t1rot);
    normalizeEulerAngles(t2rot);
    Eigen::Vector3d result_rot;

    double roll = t1ex ? t1rot(0) : t2rot(0);
    double pitch = t1ey ? t1rot(1) : t2rot(1);
    double yaw = t1ez ? t1rot(2) : t2rot(2);
    Eigen::Affine3d result;
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    q.normalize();
    result.linear() = q.toRotationMatrix();

    result.translation()(0) = t1x ? T1.translation()(0) : T2.translation()(0);
    result.translation()(1) = t1y ? T1.translation()(1) : T2.translation()(1);
    result.translation()(2) = t1z ? T1.translation()(2) : T2.translation()(2);
    return result;
}
} // namespace ndt_generic
