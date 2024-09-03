#include <ndt_localization_lamide/3d_particle_filter.hpp>
/*
 * Implementation for the 6dof pose particle filtering
 */

namespace perception_oru
{

void particle_filter_3d::initializeFromVector(
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& particles)
{

    double prob = 1.0 / ((double)particles.size());
    pcloud.resize(particles.size());
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < particles.size(); i++)
        {
            particle p;
            p.SetLikelihood(1.0);
            p.SetProbability(prob);
            p.pose = particles[i];
            p.weight = 1;
            pcloud[i] = p;
        }
    }
}

void particle_filter_3d::normalizeEulerAngles(Eigen::Vector3d& euler)
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
/**
 * Initializes the filter using normally distributed random variables with given means (m-values)
 * and standard deviations (v-values)
 */
void particle_filter_3d::initializeNormalRandom(unsigned int NumParticles,
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
                                                                bool keep_current_cloud)
{
    std::cout << "Initialize particle filter with NORMAL distribution" << std::endl;
    static std::random_device rd;
    static std::mt19937 generator(rd());

    if (!keep_current_cloud)
        pcloud.clear();
    double avg_prob = 0;
    // bool exist_previous = AverageProbability(avg_prob);
    // double init_probability = exist_previous?avg_prob : 1.0/((double)NumParticles); //Set to
    // average
    double init_probability = 1.0 / ((double)NumParticles);
    cout << "init prob: " << init_probability << endl;

    for (unsigned int i = 0; i < NumParticles; i++)
    {
        std::normal_distribution<double> distribution_x(mx, vx);
        std::normal_distribution<double> distribution_y(my, vy);
        std::normal_distribution<double> distribution_z(mz, vz);
        std::normal_distribution<double> distribution_roll(mroll, vroll);
        std::normal_distribution<double> distribution_pitch(mpitch, vpitch);
        std::normal_distribution<double> distribution_yaw(myaw, vyaw);
        double xx, yy, zz, yawyaw, rollroll, pitchpitch;
        yawyaw = distribution_yaw(generator);
        rollroll = distribution_roll(generator);
        pitchpitch = distribution_pitch(generator);
        xx = distribution_x(generator);
        yy = distribution_y(generator);
        zz = distribution_z(generator);

        particle P(rollroll, pitchpitch, yawyaw, xx, yy, zz);
        P.SetLikelihood(1.0);
        P.SetWeight(1.0);
        P.SetProbability(1.0 / (double)NumParticles);

        pcloud.push_back(P);
    }
    // cout<<"Initialized cloud, size now: "<<pcloud.size()<<endl;
}

void particle_filter_3d::initializeUniformRandom(unsigned int NumParticles,
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
                                                                 bool keep_current_cloud)
{
    std::cout << "Initialize particle filter with UNIFORM distribution" << std::endl;
    static std::random_device rd;
    static std::mt19937 generator(rd());

    if (!keep_current_cloud)
    {
        pcloud.clear();
    }

    // double init_probability = 1.0 / ((double)NumParticles);
    // cout << "init prob: " << init_probability << endl;

    for (unsigned int i = 0; i < NumParticles; i++)
    {
        std::uniform_real_distribution<double> distribution_x(mx - 0.5 * dx, mx + 0.5 * dx);
        std::uniform_real_distribution<double> distribution_y(my - 0.5 * dy, my + 0.5 * dy);
        std::uniform_real_distribution<double> distribution_z(mz - 0.5 * dz, mz + 0.5 * dz);
        std::uniform_real_distribution<double> distribution_roll(mroll - 0.5 * droll,
                                                                 mroll + 0.5 * droll);
        std::uniform_real_distribution<double> distribution_pitch(mpitch - 0.5 * dpitch,
                                                                  mpitch + 0.5 * dpitch);
        std::uniform_real_distribution<double> distribution_yaw(myaw - 0.5 * dyaw,
                                                                myaw + 0.5 * dyaw);
        double xx, yy, zz, yawyaw, rollroll, pitchpitch;
        yawyaw = distribution_yaw(generator);
        rollroll = distribution_roll(generator);
        pitchpitch = distribution_pitch(generator);
        xx = distribution_x(generator);
        yy = distribution_y(generator);
        zz = distribution_z(generator);

        particle P(rollroll, pitchpitch, yawyaw, xx, yy, zz);
        P.SetLikelihood(1.0);
        P.SetWeight(1.0);
        P.SetProbability(1.0 / (double)NumParticles);

        pcloud.push_back(P);
    }
}

void particle_filter_3d::initializeNormalRandom(unsigned int NumParticles,
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
                                                                double init_prob)
{
    std::cout << "Initialize particle filter with NORMAL distribution" << std::endl;
    // std::default_random_engine generator;
    static std::random_device rd;
    static std::mt19937 generator(rd());

    pcloud.clear();

    cout << "init prob: " << init_prob << endl;

    for (unsigned int i = 0; i < NumParticles; i++)
    {
        std::normal_distribution<double> distribution_x(mx, sqrt(vx));
        std::normal_distribution<double> distribution_y(my, sqrt(vy));
        std::normal_distribution<double> distribution_z(mz, sqrt(vz));
        std::normal_distribution<double> distribution_roll(mroll, sqrt(vroll));
        std::normal_distribution<double> distribution_pitch(mpitch, sqrt(vpitch));
        std::normal_distribution<double> distribution_yaw(myaw, sqrt(vyaw));
        double xx, yy, zz, yawyaw, rollroll, pitchpitch;
        yawyaw = distribution_yaw(generator);
        rollroll = distribution_roll(generator);
        pitchpitch = distribution_pitch(generator);
        xx = distribution_x(generator);
        yy = distribution_y(generator);
        zz = distribution_z(generator);

        particle P(rollroll, pitchpitch, yawyaw, xx, yy, zz);
        P.SetLikelihood(1.0);
        P.SetProbability(init_prob);
        pcloud.push_back(P);
    }

    // cout<<"Initialized cloud, size now: "<<pcloud.size()<<endl;
}

double particle_filter_3d::getEffectiveParticles()
{
    double sq_sum = 0;
    for (unsigned i = 0; i < pcloud.size(); i++)
    {
        sq_sum += pcloud[i].probability * pcloud[i].probability;
    }
    return 1.0 / sq_sum;
}

double particle_filter_3d::getEffectiveParticleRatio()
{
    return getEffectiveParticles() / (double)pcloud.size();
}

bool particle_filter_3d::MaxProbability(double& max)
{
    max = pcloud.front().GetProbability();
    for (auto const& p : pcloud)
    {
        if (p.probability > max)
            max = p.probability;
    }
}
bool particle_filter_3d::AverageProbability(double& avg)
{
    avg = 0;
    for (auto const& p : pcloud)
        avg += p.probability;

    if (pcloud.size() == 0)
        return false;
    else
    {
        avg = avg / pcloud.size();
        return true;
    }
}

/**
 * Performs the Sample Importance Resampling (SIR) algorithm for the distribution
 * The algorithm chooses the best particles (with respect to the probability) and
 * resamples these.
 *
 * You should have updated the likelihoods and normalized the distribution before running this
 * Also, it might be smart not to run this in every iteration, since the distribution looses
 *accuracy due to the "discretation"
 **/

void particle_filter_3d::Merge(particle_filter_3d& pf,
                                               bool delete_input)
{
    for (auto p : pf.pcloud)
        pcloud.push_back(p);

    normalize();
    if (delete_input)
        pf.pcloud.clear();
}

// New update implements Low Variance Resampling from probabilistic robotics
void particle_filter_3d::SIRUpdate(const unsigned int nr_particles)
{

    // Will be used to obtain a seed for the random number engine
    // Standard mersenne_twister_engine seeded with rd()
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0, 1);

    std::vector<particle, Eigen::aligned_allocator<particle>> Xt;

    int M = nr_particles == 0 ? pcloud.size() : nr_particles; // M is the desired output particles
    double r = ((double)dis(gen)) / ((double)M);
    double c = pcloud[0].probability;
    int i = 0;

    for (int m = 1; m <= M; m++)
    {
        double U = r + ((double)(m - 1)) / ((double)M);
        while (U > c)
        {
            i = i + 1;
            c = c + pcloud[i].probability;
        }
        Xt.push_back(pcloud[i]);
    }

    pcloud = Xt;
    normalize();
    // cout<<"After: "<<pcloud.size()<<endl;
}

/**
 * Performs the normalization step
 * i.e. according to updated likelyhoods the probability of each
 * particle is calculated and the whole distribution gets
 * probablity of 1
 */
void particle_filter_3d::normalize()
{
    int i;
    double summ = 0;

    for (unsigned i = 0; i < pcloud.size(); i++)
    {
        double alpha = 0;
        // pcloud[i].probability =(alpha +(1-alpha)*pcloud[i].probability)*pcloud[i].likelihood;
        pcloud[i].probability = pcloud[i].probability * pcloud[i].likelihood;
        // pcloud[i].probability *= pcloud[i].likelihood;
        summ += pcloud[i].probability;
    }
    if (summ > 0.00001)
    {
        for (i = 0; i < pcloud.size(); i++)
        {
            pcloud[i].probability = pcloud[i].probability / summ;
        }
    }
    else
    {
        std::cerr << "Total probability is zero : normalize" << std::endl;
        for (i = 0; i < pcloud.size(); i++)
        {
            pcloud[i].probability = 1.0 / (double)pcloud.size();
        }
    }
}

void particle_filter_3d::predict(Eigen::Affine3d Tmotion,
                                                 double vx,
                                                 double vy,
                                                 double vz,
                                                 double vroll,
                                                 double vpitch,
                                                 double vyaw,
                                                 const Eigen::Affine3d offset)
{
    Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0, 1, 2);

    bool transform_cloud = false;
    if (offset.translation().norm() > 0.0001 &&
        offset.rotation().eulerAngles(0, 1, 2).norm() > 0.0001)
        transform_cloud = true;

    static std::random_device rd;
    static std::mt19937 generator(rd());
    for (unsigned int i = 0; i < pcloud.size(); i++)
    {

        std::normal_distribution<double> distribution_x(tr[0], vx);
        std::normal_distribution<double> distribution_y(tr[1], vy);
        std::normal_distribution<double> distribution_z(tr[2], vz);
        std::normal_distribution<double> distribution_roll(rot[0], vroll);
        std::normal_distribution<double> distribution_pitch(rot[1], vpitch);
        std::normal_distribution<double> distribution_yaw(rot[2], vyaw);

        double yaw = distribution_yaw(generator);
        double roll = distribution_roll(generator);
        double pitch = distribution_pitch(generator);
        double x = distribution_x(generator);
        double y = distribution_y(generator);
        double z = distribution_z(generator);
        pcloud[i].pose = pcloud[i].pose * (xyzrpy2affine(x, y, z, roll, pitch, yaw));
        if (transform_cloud)
            pcloud[i].pose = offset * pcloud[i].pose;
    }
}

Eigen::Affine3d particle_filter_3d::getMean() const
{
    double mx = 0, my = 0, mz = 0;
    double roll_x = 0, roll_y = 0;
    double pitch_x = 0, pitch_y = 0;
    double yaw_x = 0, yaw_y = 0;

    Eigen::Matrix3d sumRot = Eigen::Matrix3d::Zero();

    for (unsigned int i = 0; i < pcloud.size(); i++)
    {
        Eigen::Vector3d tr = pcloud[i].pose.translation();
        mx += pcloud[i].probability * tr[0];
        my += pcloud[i].probability * tr[1];
        mz += pcloud[i].probability * tr[2];

        // Get as euler
        Eigen::Vector3d rot = pcloud[i].pose.rotation().eulerAngles(0, 1, 2);
        roll_x += pcloud[i].probability * cos(rot[0]);
        roll_y += pcloud[i].probability * sin(rot[0]);

        pitch_x += pcloud[i].probability * cos(rot[1]);
        pitch_y += pcloud[i].probability * sin(rot[1]);

        yaw_x += pcloud[i].probability * cos(rot[2]);
        yaw_y += pcloud[i].probability * sin(rot[2]);

        sumRot += pcloud[i].probability * pcloud[i].pose.rotation();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sumRot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d averageRot = svd.matrixU() * svd.matrixV().transpose();

    Eigen::Affine3d ret;
    ret.translation() = Eigen::Vector3d(mx, my, mz);
    ret.linear() = averageRot;
    return ret;
}

Eigen::Matrix3d particle_filter_3d::getCovariance() const
{
    Eigen::Vector3d mean = getMean().translation();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

    for (unsigned int i = 0; i < pcloud.size(); i++)
    {
        Eigen::Vector3d tr = pcloud[i].pose.translation();
        Eigen::Vector3d e = mean - tr;
        Eigen::Matrix3d v = e * e.transpose();
        cov += v;
    }
    cov = 1.0 / (double)pcloud.size() * cov;
    return cov;
}

// Sort Container by name function

Eigen::Affine3d particle_filter_3d::getMeanFiltered(float percent_inliers, int max)
{
    if (pcloud.size() > max)
        return getMean();
    std::sort(pcloud.begin(), pcloud.end(), SortByProbability);

    double mx = 0, my = 0, mz = 0;
    double roll_x = 0, roll_y = 0;
    double pitch_x = 0, pitch_y = 0;
    double yaw_x = 0, yaw_y = 0;

    Eigen::Matrix3d sumRot = Eigen::Matrix3d::Zero();

    double total_prob = 0;
    for (unsigned int i = 0; i < pcloud.size() * percent_inliers; i++)
        total_prob += pcloud[i].probability;
    if (fabs(total_prob) < 0.00001)
        std::cerr << "Total probability zero: get mean" << endl;

    for (unsigned int i = 0; i < pcloud.size() * percent_inliers; i++)
    {
        Eigen::Vector3d tr = pcloud[i].pose.translation();
        mx += pcloud[i].probability / total_prob * tr[0];
        my += pcloud[i].probability / total_prob * tr[1];
        mz += pcloud[i].probability / total_prob * tr[2];

        // Get as euler
        Eigen::Vector3d rot = pcloud[i].pose.rotation().eulerAngles(0, 1, 2);
        roll_x += pcloud[i].probability / total_prob * cos(rot[0]);
        roll_y += pcloud[i].probability / total_prob * sin(rot[0]);

        pitch_x += pcloud[i].probability / total_prob * cos(rot[1]);
        pitch_y += pcloud[i].probability / total_prob * sin(rot[1]);

        yaw_x += pcloud[i].probability / total_prob * cos(rot[2]);
        yaw_y += pcloud[i].probability / total_prob * sin(rot[2]);

        sumRot += pcloud[i].probability / total_prob * pcloud[i].pose.rotation();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sumRot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d averageRot = svd.matrixU() * svd.matrixV().transpose();

    Eigen::Affine3d ret;
    ret.translation() = Eigen::Vector3d(mx, my, mz);
    ret.linear() = averageRot;
    return ret;
}



class Stat
{
public:
    Stat(const std::string& name, bool angle = false)
    : name_(name)
    , angle_(angle)
    {
        max_val_ = std::numeric_limits<double>::min();
        min_val_ = std::numeric_limits<double>::max();
    }
    void compare(const particle& particle, double val)
    {
        if(not angle_)
        {
            if(val > max_val_)
            {
                max_val_ = val;
                max_particle_ = particle;
            }
            if (val < min_val_)
            {
                min_val_ = val;
                min_particle_ = particle;
            }
        }
        else
        {
            while(val > M_PI / 2.0)
            {
                val = val - M_PI;
            }
            if (fabs(val) > max_val_)
            {
                max_val_ = val;
                max_particle_ = particle;
            }
            if (fabs(val) < min_val_)
            {
                min_val_ = val;
                min_particle_ = particle;
            }
        }
    }
    void print()
    {
        std::cout << name_ << " max: " << max_val_ << std::endl;
        printParticle(max_particle_);
        std::cout << name_ << " min: " << min_val_ << std::endl;
        printParticle(min_particle_);
    }
    void printParticle(particle particle)
    {
        double x, y, z, rr, rp, ry, w, l, p;
        particle.GetXYZ(x, y, z);
        particle.GetRPY(rr, rp, ry);
        l = particle.GetLikelihood();
        p = particle.GetProbability();
        w = particle.GetWeight();

        std::cout << x << " ";
        std::cout << y << " ";
        std::cout << z << " ";
        std::cout << rr << " ";
        std::cout << rp << " ";
        std::cout << ry << " ";
        std::cout << w << " ";
        std::cout << l << " ";
        std::cout << p << " ";
        std::cout << std::endl;
    }

private:
    std::string name_;
    double min_val_;
    double max_val_;
    particle min_particle_;
    particle max_particle_;
    bool angle_;
};

void particle_filter_3d::printStatistics()
{
    Stat roll("roll", true);
    Stat pitch("pitch", true);
    Stat yaw("yaw", true);
    Stat x("x");
    Stat y("y");
    Stat z("z");
    Stat w("w");
    Stat l("l");
    Stat p("p");

    for (unsigned int i = 0; i < pcloud.size(); i++)
    {
        double tx, ty, tz, rr, rp, ry, pw, pl, pp;
        pcloud[i].GetXYZ(tx, ty, tz);
        pcloud[i].GetRPY(rr, rp, ry);
        pl = pcloud[i].GetLikelihood();
        pp = pcloud[i].GetProbability();
        pw = pcloud[i].GetWeight();

        roll.compare(pcloud[i], rr);
        pitch.compare(pcloud[i], rp);
        yaw.compare(pcloud[i], ry);
        x.compare(pcloud[i], tx);
        y.compare(pcloud[i], ty);
        z.compare(pcloud[i], tz);
        w.compare(pcloud[i], pw);
        l.compare(pcloud[i], pl);
        p.compare(pcloud[i], pp);
    }

    std::cout << "Particle filter cloud statistics" << std::endl;
    roll.print();
    pitch.print();
    yaw.print();
    x.print();
    y.print();
    z.print();
    w.print();
    l.print();
    p.print();
}

} // namespace perception_oru