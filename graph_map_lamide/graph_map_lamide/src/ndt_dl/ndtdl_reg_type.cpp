#include "graph_map_lamide/ndt_dl/ndtdl_reg_type.h"

#include "graph_map_lamide/ndt_dl/point_curv3.h"

namespace perception_oru
{
namespace graph_map
{

/* ----------- Parameters ------------*/
NDTDLRegParam::~NDTDLRegParam()
{
}

NDTDLRegType::NDTDLRegType(RegParamPtr paramptr) : registrationType(paramptr)
{

    NDTDLRegParamPtr param_ptr =
        boost::dynamic_pointer_cast<NDTDLRegParam>(paramptr); // Should not be NULL
    if (param_ptr != NULL)
    {

        resolutions_ = param_ptr->resolutions;
        resolution_local_factors_ = param_ptr->resolution_local_factors;
        multires_ = param_ptr->multires;
        SoftConstraints_ = param_ptr->SoftConstraints;

        // Transfer all parameters from param to this class
        cout << "Created registration type for ndtdl" << endl;
        if (SoftConstraints_)
            matcher3D_ = new NDTMatcherD2DNSC;
        else
            matcher3D_ = new NDTMatcherD2DN();

        matcher2D_ITR_MAX_ = param_ptr->matcher2D_ITR_MAX;
        matcher2D_step_control_ = param_ptr->matcher2D_step_control;
        matcher2D_n_neighbours_ = param_ptr->matcher2D_n_neighbours;

        matcher3D_->ITR_MAX = matcher2D_ITR_MAX_;
        matcher3D_->step_control = matcher2D_step_control_;
        matcher3D_->n_neighbours = matcher2D_step_control_;

        matcher3D_2_ = new NDTMatcherD2D();

        matcher3D_2_->ITR_MAX = matcher2D_ITR_MAX_;
        matcher3D_2_->step_control = matcher2D_step_control_;
        matcher3D_2_->n_neighbours = matcher2D_n_neighbours_;
    }
    else
        cerr << "ndtd2d registrator has NULL parameters" << endl;
}

NDTDLRegParam::NDTDLRegParam() : registrationParameters()
{
}

void NDTDLRegParam::GetParametersFromRos()
{
    registrationParameters::GetParametersFromRos();
    ros::NodeHandle nh("~");
    int matcher2D_n_neighbours = 2;

    nh.param("itr_max", matcher2D_ITR_MAX, 25);
    nh.param<bool>("step_control", matcher2D_step_control, true);
    nh.param<bool>("multires", multires, false);
    nh.param<bool>("soft_constraints", SoftConstraints, false);
    nh.param("matcher_neighbours", matcher2D_n_neighbours, 2);

    int n = 3;
    nh.param("map_dimensions", n, 3);

    resolution_local_factors.resize(n);
    for (int i = 0; i < n; i++)
        nh.param("resolution_local_factor_" + std::to_string(i + 1), resolution_local_factors[i],
                 2.0);

    resolutions.resize(n);
    for (int i = 0; i < n; i++)
        nh.param("resolution_" + std::to_string(i + 1), resolutions[i], 1.0);
}

NDTDLRegType::~NDTDLRegType()
{
}

bool NDTDLRegType::Register(MapTypePtr maptype,
                            Eigen::Affine3d& Tnow,
                            pcl::PointCloud<pcl::PointXYZL>& cloud,
                            Eigen::MatrixXd& Tcov)
{

    cout << "registration of PointCloud<pcl::PointXYZL> is disabled until it is implemented for map "
            "of type (NDTDL): "
         << maptype->GetMapName() << endl;
    return true; // Remove when registration has been implemented

    if (!enable_registraiton || !maptype->Initialized())
    {
        cout << "Registration disabled - motion based on odometry" << endl;

        return false;
    }
    else
    {
        NDTMapPtr ndt_map_ptr = boost::dynamic_pointer_cast<NDTMapType>(maptype);
        if (ndt_map_ptr != NULL)
        {
            std::cout << "Warning: no point in running the N registration with a 1 map - do nothing"
                      << std::endl;
            return false;
        }
        else
        {
            NDTDLMapPtr MapPtr = boost::dynamic_pointer_cast<NDTDLMapType>(maptype);
            // Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
            if (MapPtr != NULL)
            {
                // Add PointXYZ segmentation.
                std::cout << "Implement segm. for PointXYZ with N registration" << std::endl;
                return false;
            }
        }
    }
}

bool NDTDLRegType::Register(MapTypePtr maptype,
                            Eigen::Affine3d& Tnow,
                            std::vector<pcl::PointCloud<pcl::PointXYZL>>& segm_clouds,
                            Eigen::MatrixXd& Tcov)
{

    std::vector<NDTMap*> globalMap;
    // Create local map
    ///
    NDTMapPtr MapPtr = boost::dynamic_pointer_cast<NDTMapType>(maptype);
    if (MapPtr != NULL)
    {
        globalMap.push_back(MapPtr->GetNDTMap());
        std::cout << "Warning: running N registration with a 1 map" << std::endl;
    }
    else
    {
        // Need to handle the other maps...
        NDTDLMapPtr DLMapPtr = boost::dynamic_pointer_cast<NDTDLMapType>(maptype);
        if (DLMapPtr != NULL)
        {
            globalMap = DLMapPtr->GetNDTMaps();
        }
        else
        {
            std::cerr << "NDTD2DRegType::Register don't handle this maptype : "
                      << maptype->GetMapName() << std::endl;
            return false;
        }
    }

    std::vector<NDTMap*> ndlocals(segm_clouds.size());
    for (size_t i = 0; i < ndlocals.size(); i++)
        ndlocals[i] = NULL;

    bool registration_status = true;
    if (multires_)
    {
        std::cerr << "multires!" << std::endl;
        for (int i = 1; i >= 0; i--)
        { // should be i=2
            for (size_t j = 0; j < segm_clouds.size(); j++)
            {
                if (ndlocals[j] != NULL)
                {
                    delete ndlocals[j];
                }
                ndlocals[j] = new NDTMap(new perception_oru::LazyGrid(
                    resolutions_[j] * resolution_local_factors_[j] * pow(2, i)));
                ndlocals[j]->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
                ndlocals[j]->loadPointCloud(segm_clouds[j], sensor_range);
                ndlocals[j]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
            }
            if (SoftConstraints_)
                std::cerr << "Soft constraints disabled for multi-resolution registration" << endl;
            else
                registration_status = matcher3D_->match(globalMap, ndlocals, Tnow, true);

            // registration_status= matcher3D_->match( globalMap, ndlocals,Tnow,true);
        }
    }
    else if (!multires_)
    {
        for (size_t j = 0; j < segm_clouds.size(); j++)
        {
            if (ndlocals[j] != NULL)
            {
                delete ndlocals[j];
            }
            ndlocals[j] = new NDTMap(
                new perception_oru::LazyGrid(resolutions_[j] * resolution_local_factors_[j]));
            ndlocals[j]->guessSize(0, 0, 0, sensor_range, sensor_range, map_size_z);
            ndlocals[j]->loadPointCloud(segm_clouds[j], sensor_range);
            ndlocals[j]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
        }

        if (!registration2d_)
        {
            if (SoftConstraints_)
            {
                registration_status = dynamic_cast<NDTMatcherD2DNSC*>(matcher3D_)
                                          ->match(globalMap, ndlocals, Tnow, Tcov);
                std::cout << "Registration returned:" << std::boolalpha << registration_status
                          << std::endl;
            }
            else
                registration_status = matcher3D_->match(globalMap, ndlocals, Tnow, true);
        }
        else
        {
            std::cerr << "2D registration not implemented for dl" << std::endl;
        }
    }
    status_ = matcher3D_->status_;
    score_ = matcher3D_->finalscore;
    // status_=matcher3D_2_->status_;
    // score_=matcher3D_2_->finalscore;

    for (size_t j = 0; j < ndlocals.size(); j++)
    {
        if (ndlocals[j] != NULL)
        {
            delete ndlocals[j];
        }
    }
    return registration_status;
}

bool NDTDLRegType::Register(MapTypePtr maptype,
                            Eigen::Affine3d& Tnow,
                            pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                            Eigen::MatrixXd& Tcov)
{
    // Separate the point cloud into several.
    std::vector<pcl::PointCloud<pcl::PointXYZL>> segm_clouds;
    segmentPointCurvature3(Tnow, cloud, segm_clouds);
    return this->Register(maptype, Tnow, segm_clouds, Tcov);
}
std::string NDTDLRegType::ToString()
{
    std::stringstream ss;
    ss << registrationType::ToString() << endl;
    ss << "NDT-DL registraiton: " << endl;
    ss << "matcher2D_ITR_MAX " << matcher2D_ITR_MAX_ << endl;
    ss << "matcher2D_step_control: " << std::boolalpha << matcher2D_step_control_ << endl;
    ss << "matcher2D_n_neighbours: " << matcher2D_n_neighbours_ << endl;
    ss << "resolutions: [";
    for (int i = 0; i < resolutions_.size(); i++)
        ss << resolutions_[i] << ", ";
    ss << "]" << endl;
    ss << "resolution local factor: [";
    for (int i = 0; i < resolution_local_factors_.size(); i++)
        ss << resolution_local_factors_[i] << ", ";
    ss << "]" << endl;
    ss << "SoftConstraints: " << std::boolalpha << SoftConstraints_ << endl;
    ss << "multiresolution: " << std::boolalpha << multires_ << endl;
    return ss.str();
}

} // namespace graph_map
} // namespace perception_oru
