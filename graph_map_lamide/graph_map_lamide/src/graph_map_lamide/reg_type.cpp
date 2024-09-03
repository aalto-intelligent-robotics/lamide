#include "graph_map_lamide/reg_type.h"


namespace perception_oru
{
namespace graph_map
{

bool registrationType::RegisterScan(pcl::PointCloud<pcl::PointXYZL>& target,
                                    Eigen::Affine3d& Tnow,
                                    pcl::PointCloud<pcl::PointXYZL>& src,
                                    Eigen::MatrixXd& Tcov)
{
    if (!enable_registraiton || target.points.empty() || src.points.empty())
    {
        cout << "Registration disabled, target: " << target.size() << ", src: " << src.size()
             << "enable reg: " << std::boolalpha << enable_registraiton << endl;
        return true;
    }

    Eigen::Affine3d Tinit = use_initial_guess ? Tnow : Eigen::Affine3d::Identity();
    bool registration_return = Register(
        target, Tinit, src,
        Tcov); // please overwrite this function or use one of the existing registration methods...
    Eigen::Affine3d diff = Tnow.inverse() * Tinit;
    Eigen::Vector3d euler = diff.rotation().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(euler);
    double angle_rot_norm = euler.norm();

    if (registration_return == false)
    { // unsuccesfull registration
        if (status_ == MAX_ITR)
        {
            cout << "Registration max iterations" << endl;
            Tnow = Tinit;
        }
        else
            cerr << "Registration Internal failure" << endl;
        failed_registrations++;
        return false;
    }
    else
    {
        if (check_consistency && diff.translation().norm() > max_translation_norm)
        {
            cerr << "Registration failure: Translation far from prediction, "
                 << diff.translation().norm() << "m  >  " << max_translation_norm << endl;
            failed_registrations++;
            return false;
        }
        else if (check_consistency && (angle_rot_norm > max_rotation_norm))
        {
            cerr << "Registration failure: Orientation too far from prediction, " << angle_rot_norm
                 << "rad  >  " << max_rotation_norm << "rad" << endl;
            failed_registrations++;
            return false;
        }
        else
        {
            Tnow = Tinit; // All well, translation and rotation feasible
            cout << "Registration success" << endl;
            succesfull_registrations++;
            return true;
        }
    }
}
bool registrationType::RegisterScan(MapTypePtr maptype,
                                    Eigen::Affine3d& Tnow,
                                    pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6);
    return RegisterScan(maptype, Tnow, cloud, cov);
}

bool registrationType::RegisterScan(MapTypePtr maptype,
                                    Eigen::Affine3d& Tnow,
                                    pcl::PointCloud<pcl::PointXYZL>& cloud,
                                    Eigen::MatrixXd& Tcov)
{
    if (!enable_registraiton || !maptype->Initialized())
    {
        cout << "Registration disabled" << endl;
        return true;
    }

    Eigen::Affine3d Tinit = use_initial_guess ? Tnow : Eigen::Affine3d::Identity();
    bool registration_return = Register(
        maptype, Tinit, cloud,
        Tcov); // please overwrite this function or use one of the existing registration methods...
    Eigen::Affine3d diff = Tnow.inverse() * Tinit;
    Eigen::Vector3d euler = diff.rotation().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(euler);
    double angle_rot_norm = euler.norm();

    if (registration_return == false)
    { // unsuccesfull registration
        if (status_ == MAX_ITR)
        {
            cout << "Registration max iterations" << endl;
            Tnow = Tinit;
        }
        else
            cerr << "Registration Internal failure" << endl;
        failed_registrations++;
        return false;
    }
    else
    { // registration succesfull
        // Kcout<<"registraiotn diff="<<diff.translation()<<endl;
        if (check_consistency && diff.translation().norm() > max_translation_norm)
        {
            cerr << "Registration failure: Translation far from prediction, "
                 << diff.translation().norm() << "m  >  " << max_translation_norm << endl;
            failed_registrations++;
            return false;
        }
        else if (check_consistency && (angle_rot_norm > max_rotation_norm))
        {
            cerr << "Registration failure: Orientation too far from prediction, " << angle_rot_norm
                 << "rad  >  " << max_rotation_norm << "rad" << endl;
            failed_registrations++;
            return false;
        }
        else
        {
            Tnow = Tinit; // All well, translation and rotation feasible
            cout << "Registration success" << endl;
            succesfull_registrations++;
            return true;
        }
    }
}
bool registrationType::RegisterScan(MapTypePtr maptype,
                                    Eigen::Affine3d& Tnow,
                                    std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds)
{
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6);
    return RegisterScan(maptype, Tnow, clouds, cov);
}
bool registrationType::RegisterScan(MapTypePtr maptype,
                                    Eigen::Affine3d& Tnow,
                                    std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                                    Eigen::MatrixXd& Tcov)
{
    if (clouds.empty() || maptype == NULL)
    {
        cout << "Cloud or map is empty" << endl;
        return false;
    }

    if (!enable_registraiton)
    {
        cout << "Registration disabled" << endl;
        return true;
    }

    Eigen::Affine3d Tinit = use_initial_guess ? Tnow : Eigen::Affine3d::Identity();
    bool registration_return = Register(
        maptype, Tinit, clouds,
        Tcov); // please overwrite this function or use one of the existing registration methods...
    Eigen::Affine3d diff = Tnow.inverse() * Tinit;
    Eigen::Vector3d euler = diff.rotation().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(euler);
    double angle_rot_norm = euler.norm();
    if (registration_return == false)
    { // unsuccesfull registration
        cerr << "Registration Internal failure" << endl;
        failed_registrations++;
        return false;
    }
    else
    { // registration succesfull
        if (check_consistency && diff.translation().norm() > max_translation_norm)
        {
            cerr << "Registration failure: Translation far from prediction, "
                 << diff.translation().norm() << "m  >  " << max_translation_norm << endl;
            failed_registrations++;
            return false;
        }
        else if (check_consistency && (angle_rot_norm > max_rotation_norm))
        {
            cerr << "Registration failure: Orientation too far from prediction, (also euler not "
                    "normalized!!)"
                 << angle_rot_norm << "rad  >  " << max_rotation_norm << "rad" << endl;
            failed_registrations++;
            return false;
        }
        else
        {
            Tnow = Tinit; // All well, translation and rotation feasible
            cout << "Registration succesfull" << endl;
            succesfull_registrations++;
            return true;
        }
    }
}
bool registrationType::Map2MapRegistration(MapTypePtr target,
                                           MapTypePtr source,
                                           Eigen::Affine3d& T,
                                           double& match_score)
{
    if (!enable_registraiton)
    {
        cout << "Registration disabled" << endl;
        return true;
    }
    if (target == NULL || source == NULL)
    {
        return false;
        cout << "Cannot register maps, NULL map pointer" << endl;
    }

    Eigen::Affine3d Tinit = T;
    bool registration_return = RegisterMap2Map(target, source, Tinit, match_score);
    Eigen::Affine3d diff = Tinit * T.inverse();
    Eigen::Vector3d euler = diff.rotation().eulerAngles(0, 1, 2);
    ndt_generic::normalizeEulerAngles(euler);
    if (registration_return == false)
    { // unsuccesfull registration
        cerr << "Map2Map internal failure" << endl;
        failed_registrations++;
        return false;
    }
    else
    { // registration succesfull
        if (check_consistency && diff.translation().norm() > max_translation_norm)
        {
            cerr << "Map2Map registration failure: Translation far from prediction, "
                 << diff.translation().norm() << "m  >  " << max_translation_norm << endl;
            failed_registrations++;
            return false;
        }
        else if (check_consistency && (euler.norm() > max_rotation_norm))
        {
            cerr << "Map2Map registration failure: Orientation too far from prediction, (also "
                    "euler not normalized!!)"
                 << euler.norm() << "rad  >  " << max_rotation_norm << "rad" << endl;
            failed_registrations++;
            return false;
        }
        else
        {
            T = Tinit; // All well, translation and rotation feasible
            cout << "Map2Map registration succesfull" << endl;
            succesfull_registrations++;
            return true;
        }
    }
}
/* -------Registration type---------------- */
registrationType::registrationType(RegParamPtr regparam) : OutputCov(6, 6)
{
    if (regparam != NULL)
    {
        sensorPose_ = regparam->sensor_pose;
        cout << "created registration type" << endl;
        enable_registraiton = regparam->enable_registration;
        registration2d_ = regparam->registration2d;
        check_consistency = regparam->check_consistency;
        max_translation_norm = regparam->max_translation_norm;
        max_rotation_norm = regparam->max_rotation_norm;
        rotation_registration_delta = regparam->rotation_registration_delta;
        sensor_range = regparam->sensor_range;
        map_size_z = regparam->map_size_z;
        use_initial_guess = regparam->use_initial_guess;
        do_soft_constraints_ = regparam->do_soft_constraints;
        calculate_cov_ = regparam->calculate_cov;
        sensorPose_ = regparam->sensor_pose;
        failed_registrations = 0;
        succesfull_registrations = 0;
        cout << "sucessfully applied registration parameters" << endl;
    }
    else
        cerr << "Registration parameters cannot be applied to registrator as parameter object does "
                "not exist"
             << endl;
}
registrationType::~registrationType()
{
}
std::string registrationType::ToString()
{
    std::stringstream ss;
    ss << endl << "registration type:" << endl;
    ss << "enableRegistration: " << std::boolalpha << enable_registraiton << endl;
    if (enable_registraiton)
    {
        ss << "registration limited to 2d: " << std::boolalpha << registration2d_ << endl;
        ss << "Check consistency: " << std::boolalpha << check_consistency << endl;
        ss << "max registration distances(translation,rotation): (" << max_translation_norm << ","
           << max_rotation_norm << ")" << endl;
        ss << "Maximum sensor range: " << sensor_range << endl;
        ss << "Map size z: " << map_size_z << endl;
        ss << "sensor position offset: (x,y,z): (" << sensorPose_.translation().transpose()(0)
           << "," << sensorPose_.translation().transpose()(1) << ","
           << sensorPose_.translation().transpose()(2) << ")" << endl;
    }
    return ss.str();
}

/* -------Parameters---------------- */
registrationParameters::registrationParameters()
{
}

registrationParameters::~registrationParameters()
{
}

void registrationParameters::GetParametersFromRos()
{
    bool render_GT_map;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("enable_registration", enable_registration, true);
    nh.param("registration_2D", registration2d, false);
    nh.param("check_consistency", check_consistency, true);
    nh.param("max_range", sensor_range, 130.0);
    nh.param("size_z_meters", map_size_z, 20.0);
    nh.param("max_translation_norm", max_translation_norm, 0.4);
    nh.param("max_rotation_norm", max_rotation_norm, M_PI / 4);
    nh.param("renderGTmap", render_GT_map, false);
    nh.param("do_soft_constraints", do_soft_constraints, false);
    nh.param("calculate_cov", calculate_cov, false);

    if (render_GT_map)
        enable_registration = false;
}

} // namespace graph_map
} // namespace perception_oru
