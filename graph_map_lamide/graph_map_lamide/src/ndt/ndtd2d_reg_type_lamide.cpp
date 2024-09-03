#include "graph_map_lamide/ndt/ndtd2d_reg_type_lamide.h"
#include <graph_map_lamide/ndt_dl/point_curv3.h>

namespace perception_oru
{
namespace graph_map
{

NDTLamideRegType::NDTLamideRegType(RegParamPtr paramptr) : registrationType(paramptr)
{

    NDTLamideRegParamPtr param_ptr =
        boost::dynamic_pointer_cast<NDTLamideRegParam>(paramptr); // Should not be NULL
    if (param_ptr != NULL)
    {
        resolution_ = param_ptr->resolution;
        resolutionLocalFactor_ = param_ptr->resolution_local_factor;
        multires_ = param_ptr->multires;
        // SoftConstraints_=param_ptr->SoftConstraints;

        if (registration2d_)
        {
            if (do_soft_constraints_)
            {
                matcher2D_ = new NDTMatcherD2DSC_2D();
            }
            else
            {
                matcher2D_ = new NDTMatcherD2D_2D();
            }

            matcher2D_->ITR_MAX = param_ptr->matcher2D_ITR_MAX;
            matcher2D_->step_control = param_ptr->matcher2D_step_control;
            matcher2D_->n_neighbours = param_ptr->matcher2D_n_neighbours;
        }
        else
        {
            if (do_soft_constraints_)
            {
                std::cout << "Soft Constraints do not work with Lamide!" << std::endl;
            }
            else
            {
                cout << "Create matcher LAMIDE ndtd2d" << endl;
                matcher3D_ = new NDTMatcherLamide();
            }

            matcher3D_->ITR_MAX = param_ptr->matcher2D_ITR_MAX;
            matcher3D_->step_control = param_ptr->matcher2D_step_control;
            matcher3D_->n_neighbours = param_ptr->matcher2D_n_neighbours;
        }
    }
    else
        cerr << "ndtd2d registrator has NULL parameters" << endl;
}

NDTLamideRegType::~NDTLamideRegType()
{
}

bool NDTLamideRegType::Register(MapTypePtr maptype,
                             Eigen::Affine3d& Tnow,
                             pcl::PointCloud<pcl::PointXYZL>& cloud,
                             Eigen::MatrixXd& Tcov)
{
    std::cout << "NDTD2D lamide reg" << std::endl;
    NDTMap* globalMap;

    // Create local map
    ///
    NDTMapPtr MapPtr = boost::dynamic_pointer_cast<NDTMapType>(maptype);
    if (MapPtr != NULL)
    {
        globalMap = MapPtr->GetNDTMap();
    }
    else
    {
        // Need to handle the other maps...
        NDTDLMapPtr DLMapPtr = boost::dynamic_pointer_cast<NDTDLMapType>(maptype);
        if (DLMapPtr != NULL)
        {
            std::cerr << "got a DL map : " << maptype->GetMapName() << std::endl;
            globalMap = DLMapPtr->GetNDTMaps()[0];
        }
        else
        {
            std::cerr << "NDTLamideRegType::Register don't handle this maptype : "
                      << maptype->GetMapName() << std::endl;
            return false;
        }
    }

    bool registration_status = true;
    if (multires_)
    {
        for (int i = 1; i >= 0; i--)
        { // should be i=2
            //  cout<<"REG resolution factor="<<i<<endl;
            delete ndlocal;
            ndlocal = NULL;
            ndlocal = new NDTMap(new perception_oru::LazyGrid(resolution_ * pow(2, i)));
            // rception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution_*pow(2,i)));
            ndlocal->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
            ndlocal->loadPointCloud(cloud, sensor_range);
            ndlocal->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
            if (do_soft_constraints_)
                std::cerr << "Soft constraints disabled for multi-resolution registration" << endl;
            if (registration2d_)
                registration_status = matcher2D_->match(*globalMap, *ndlocal, Tnow, true);
            else
                registration_status = matcher3D_->match(*globalMap, *ndlocal, Tnow, true);
        }
    }
    else if (!multires_)
    {
        // cout<<"regular registration"<<endl;
        delete ndlocal;
        ndlocal = NULL;
        ndlocal = new NDTMap(new perception_oru::LazyGrid(resolution_ * resolutionLocalFactor_));
        ndlocal->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
        ndlocal->loadPointCloud(cloud, sensor_range);
        ndlocal->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
        if (do_soft_constraints_ && !registration2d_)
        {
            cout << "SC" << endl;
            registration_status =
                dynamic_cast<NDTMatcherD2DSC*>(matcher3D_)->match(*globalMap, *ndlocal, Tnow, Tcov);
            // Cov = dynamic_cast<NDTMatcherD2DSC*>(matcher3D_)->Cov;
        }
        else if (do_soft_constraints_ && registration2d_)
        {
            Eigen::Matrix3d Tcov2d = ndt_generic::Cov6dTo3d(Tcov);
            registration_status = dynamic_cast<NDTMatcherD2DSC_2D*>(matcher2D_)
                                      ->match(*globalMap, *ndlocal, Tnow, Tcov2d);
        }
        else if (!do_soft_constraints_ && registration2d_)
            registration_status = matcher2D_->match(*globalMap, *ndlocal, Tnow, true);
        else if (!do_soft_constraints_ && !registration2d_)
        {
            cout << "NDT D2D 3D - LaMiDE" << endl;
            registration_status = matcher3D_->match(*globalMap, *ndlocal, Tnow, true);
        }
    }
    if (calculate_cov_)
    {
        if (!registration2d_)
        {
            std::cout << "compute covariance" << std::endl;
            Eigen::MatrixXd Covest;
            bool cov_correct = matcher3D_->covariance(*globalMap, *ndlocal, Tnow, Covest);
            if (cov_correct)
                OutputCov = Covest;
        }
    }

    if (registration2d_)
    {
        status_ = matcher2D_->status_;
        score_ = matcher2D_->finalscore;
    }
    else
    {
        status_ = matcher3D_->status_;
        score_ = matcher3D_->finalscore;
    }

    return registration_status;
}
bool NDTLamideRegType::Register(MapTypePtr maptype,
                             Eigen::Affine3d& Tnow,
                             std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                             Eigen::MatrixXd& Tcov)
{

    if (clouds.empty())
        return false;
    return Register(maptype, Tnow, clouds[0], Tcov);
}

bool NDTLamideRegType::RegisterMap2Map(MapTypePtr map_prev,
                                    MapTypePtr map_next,
                                    Eigen::Affine3d& Tdiff,
                                    double& match_score)
{

    Eigen::Affine3d Tinit = Tdiff;
    bool match_succesfull;
    NDTMap* prev = (boost::dynamic_pointer_cast<NDTMapType>(map_prev))->GetNDTMap();
    NDTMap* next = (boost::dynamic_pointer_cast<NDTMapType>(map_next))->GetNDTMap();
    if (registration2d_)
    {
        match_succesfull = matcher2D_->match(*prev, *next, Tinit, true);
    }
    else if (!registration2d_)
    {
        match_succesfull = matcher3D_->match(*prev, *next, Tinit, true);
    }
    if (match_succesfull)
    {
        match_score = matcher3D_->finalscore;
        Tdiff = Tinit;
        cout << "final score=" << match_score << endl;
        cout << "diff after reg-trans=\n" << Tdiff.translation() << endl;
        cout << "diff after reg-rot=\n=" << Tdiff.linear() << endl;
    }
    else
    {
        cout << "error registering maps" << endl;
        match_score = 0;
    }

    return match_succesfull;
}

bool NDTLamideRegType::Register(pcl::PointCloud<pcl::PointXYZL>& target,
                             Eigen::Affine3d& Tnow,
                             pcl::PointCloud<pcl::PointXYZL>& src,
                             Eigen::MatrixXd& Tcov)
{
    boost::shared_ptr<NDTMap> ndttar;
    boost::shared_ptr<NDTMap> ndtsrc;
    bool registration_status = true;
    if (multires_)
    {
        cout << "multires" << endl;
        for (int i = 1; i >= 0; i--)
        { // should be i=2
            ndttar = boost::shared_ptr<NDTMap>(
                new NDTMap(new perception_oru::LazyGrid(resolution_ * pow(2, i))));
            ndtsrc = boost::shared_ptr<NDTMap>(
                new NDTMap(new perception_oru::LazyGrid(resolution_ * pow(2, i))));
            // rception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution_*pow(2,i)));
            ndtsrc->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
            ndtsrc->loadPointCloud(src, sensor_range);
            ndtsrc->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

            ndttar->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
            ndttar->loadPointCloud(target, sensor_range);
            ndttar->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
            if (do_soft_constraints_)
                std::cerr << "Soft constraints disabled for multi-resolution registration" << endl;
            if (registration2d_)
                registration_status = matcher2D_->match(*ndttar, *ndtsrc, Tnow, true);
            else
            {
                cout << "match: " << ndttar->getAllCellsNoCopy().size() << " and "
                     << ndtsrc->getAllCellsNoCopy().size() << endl;
                registration_status = matcher3D_->match(*ndttar, *ndtsrc, Tnow, true);
            }
        }
    }
    else if (!multires_)
    {

        ndttar = boost::shared_ptr<NDTMap>(new NDTMap(new perception_oru::LazyGrid(resolution_)));
        ndtsrc = boost::shared_ptr<NDTMap>(new NDTMap(new perception_oru::LazyGrid(resolution_)));
        // rception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution_*pow(2,i)));
        ndtsrc->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
        ndtsrc->loadPointCloud(src, sensor_range);
        ndtsrc->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

        ndttar->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
        ndttar->loadPointCloud(target, sensor_range);
        ndttar->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

        if (do_soft_constraints_ && !registration2d_)
        {
            std::cout << "Soft constraints do not work with Lamide!!!" << std::endl;
        }
        else if (do_soft_constraints_ && registration2d_)
        {
            Eigen::Matrix3d Tcov2d = ndt_generic::Cov6dTo3d(Tcov);
            registration_status = dynamic_cast<NDTMatcherD2DSC_2D*>(matcher2D_)
                                      ->match(*ndttar, *ndtsrc, Tnow, Tcov2d);
        }
        else if (!do_soft_constraints_ && registration2d_)
        {
            registration_status = matcher2D_->match(*ndttar, *ndtsrc, Tnow, true);
        }
        else if (!do_soft_constraints_ && !registration2d_)
        {
            cout << "NDT D2D 3D - LaMiDE" << endl;
            cout << "match: " << ndttar->getAllCellsNoCopy().size() << " and "
                 << ndtsrc->getAllCellsNoCopy().size() << endl;
            registration_status = matcher3D_->match(*ndttar, *ndtsrc, Tnow, true);
            cout << "final score: " << matcher3D_->finalscore << endl;
        }
    }
    if (calculate_cov_)
    {
        if (!registration2d_)
        {
            std::cout << "compute covariance" << std::endl;
            Eigen::MatrixXd Covest;
            bool cov_correct = matcher3D_->covariance(*ndttar, *ndtsrc, Tnow, Covest);
            if (cov_correct)
                OutputCov = Covest;
        }
    }

    if (registration2d_)
    {
        status_ = matcher2D_->status_;
        score_ = matcher2D_->finalscore;
    }
    else
    {
        status_ = matcher3D_->status_;
        score_ = matcher3D_->finalscore;
    }

    return registration_status;
}

std::string NDTLamideRegType::ToString()
{
    std::stringstream ss;
    ss << registrationType::ToString();
    if (enable_registraiton)
    {
        ss << "NDT d2d registration type:" << endl;
        ss << "resolution :" << resolution_ << endl;
        ss << "resolutionLocalFactor :" << resolutionLocalFactor_ << endl;
        // ss<<"Soft constraints :"<< std::boolalpha<<SoftConstraints_<<endl;
        if (!registration2d_)
        {
            ss << "max opt itr :" << matcher3D_->ITR_MAX << endl;
            ss << "opt step ctrl :" << std::boolalpha << matcher3D_->step_control << endl;
            ss << "d2d neighboors :" << matcher3D_->n_neighbours << endl;
        }
        else if (registration2d_)
        {
            ss << "max opt itr :" << matcher2D_->ITR_MAX << endl;
            ss << "opt step ctrl :" << std::boolalpha << matcher2D_->step_control << endl;
            ss << "d2d neighboors :" << matcher2D_->n_neighbours << endl;
        }
        ss << "multires :" << std::boolalpha << multires_ << endl;
    }
    else
        ss << "Registration disabled:" << endl;
    return ss.str();
}

/* ----------- Parameters ------------*/
NDTLamideRegParam::~NDTLamideRegParam()
{
}
NDTLamideRegParam::NDTLamideRegParam() : registrationParameters()
{
}
void NDTLamideRegParam::GetParametersFromRos()
{
    registrationParameters::GetParametersFromRos();
    cout << "derived class read from ros" << endl;
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("resolution", resolution, 0.6);
    nh.param("reg_itr_max", matcher2D_ITR_MAX, 35);
    nh.param("resolutionLocalFactor", resolution_local_factor, 1.0);
    nh.param("multires", multires, false);
    nh.param("n_neighbours", matcher2D_n_neighbours, 2);
}

} // namespace graph_map

} // namespace perception_oru
