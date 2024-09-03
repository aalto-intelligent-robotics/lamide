#include "graph_map_lamide/map_type.h"


namespace perception_oru
{
namespace graph_map
{

//: octree(0.02)
MapType::MapType()
{
    sizex_ = 0;
    sizey_ = 0;
    sizez_ = 0;
    max_range_ = 100;
    min_range_ = 0.6;
    initialized_ = false;
    enable_mapping_ = true;
    store_points_ = false;
    mapName_ = "";
}
//: octree(0.02)
MapType::MapType(MapParamPtr param)
{
    initialized_ = false;
    enable_mapping_ = param->enable_mapping_;
    sizex_ = param->sizex_;
    sizey_ = param->sizey_;
    sizez_ = param->sizez_;
    max_range_ = param->max_range_;
    min_range_ = param->min_range_;
    store_points_ = param->store_points;
    mapName_ = "";
    prototype_ = param;
}
void MapType::GetCloud(pcl::PointCloud<pcl::PointXYZL>& cloud, bool downsampled)
{
    for (int i = 0; i < input_cloud_.size(); ++i)
    {
        cloud += input_cloud_[i];
    }
}
void MapType::updateMap(const Eigen::Affine3d& Tnow,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple,
                        bool cluster)
{
    if (enable_mapping_)
    {
        stamp = cloud.header.stamp;
        update(Tnow, cloud, simple, cluster);
        pose_est_.push_back(Tnow);
        mean_obs_location_.translation() =
            (Tnow.translation() + observations_ * mean_obs_location_.translation()) /
            (observations_ + 1.0);
        observations_++;
        if (store_points_)
        {
            input_cloud_.push_back(cloud);
        }
    }
}
void MapType::updateMap(const Eigen::Affine3d& Tnow,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple,
                        bool cluster)
{
    if (clouds.empty())
    {
        return;
    }

    if (enable_mapping_)
    {
        stamp = clouds[0].header.stamp;
        update(Tnow, clouds, simple, cluster);

        if (store_points_)
            for (int i = 0; i < clouds.size(); i++)
                input_cloud_.push_back(clouds[i]);

        pose_est_.push_back(Tnow);
        mean_obs_location_.translation() =
            (Tnow.translation() + observations_ * mean_obs_location_.translation()) /
            (observations_ + 1.0);
    }
    else
        cout << "Mapping disabled" << endl;
}

bool MapType::CompoundMapsByRadius(MapTypePtr target,
                                   const Affine3d& T_source,
                                   const Affine3d& T_target,
                                   double radius)
{
    cout << "Compunding map not possible in base class" << endl;
    return false;
}
std::string MapType::ToString()
{
    std::stringstream ss;
    ss << "MapType\nInitialized: " << std::boolalpha << initialized_ << endl;
    ss << "Map Name: " << mapName_ << endl;
    ss << "Mapping enabled: " << std::boolalpha << enable_mapping_ << endl;
    ss << "Size(x,y,z): (" << sizex_ << "," << sizey_ << "," << sizez_ << ")" << endl;
    ss << "Sensor range(min/max): (" << min_range_ << "," << max_range_ << ")" << endl;
    ss << "scans: " << input_cloud_.size() << endl;
    int pnts = 0;
    for (int i = 0; i < input_cloud_.size(); ++i)
    {
        pnts += input_cloud_[i].size();
    }
    ss << "Points: " << pnts << endl;
    return ss.str();
}
MapParamPtr MapType::GetPrototype()
{
    return prototype_;
}
MapParam::MapParam(const MapParam& par)
{
    max_range_ = par.max_range_;
    min_range_ = par.min_range_;
    sizex_ = par.sizex_;
    sizey_ = par.sizey_;
    sizez_ = par.sizez_;
    enable_mapping_ = par.enable_mapping_;
}
MapParam::~MapParam()
{
}

void MapParam::GetParametersFromRos()
{
    ros::NodeHandle nh("~"); // base class parameters
    nh.param("max_range", max_range_, 100.);
    nh.param("min_range", min_range_, 0.6);
    nh.param("size_x_meters", sizex_, .50);
    nh.param("size_y_meters", sizey_, .50);
    nh.param("size_z_meters", sizez_, .10);
    nh.param("enable_mapping", enable_mapping_, true);
    cout << "read mapType parameters from ros" << endl;
    cout << ToString() << endl;
}
string MapParam::ToString()
{
    std::stringstream ss;
    ss << "Base map parameters:" << endl;
    ss << "Range(max/min)=(" << max_range_ << "/" << min_range_ << endl;
    ss << "size(x,y,z)=(" << sizex_ << "," << sizey_ << "," << sizez_ << ")";
    return ss.str();
}
} // namespace graph_map
} // namespace perception_oru
