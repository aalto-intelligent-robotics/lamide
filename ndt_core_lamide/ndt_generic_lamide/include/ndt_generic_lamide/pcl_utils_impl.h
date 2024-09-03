#ifndef PCL_UTILS_IMPL_H
#define PCL_UTILS_IMPL_H

namespace ndt_generic
{

template <class PointT>
bool UnwarpCloudSimple(velodyne_rawdata_oru::RawData& velodyneparser,
                       velodyne_msgs::VelodyneScan::ConstPtr& scan,
                       pcl::PointCloud<PointT>& cloud,
                       double min_range,
                       double max_range)
{
    cloud.clear();
    static velodyne_pointcloud_oru::PointcloudXYZIR pnts(
        max_range, min_range, "/velodyne", "/velodyne", velodyneparser.scansPerPacket(), NULL);
    pnts.setup(scan);
    for (int i = 0; i < scan->packets.size(); i++)
    {
        velodyneparser.unpack(scan->packets[i], pnts); // unpack the raw data
    }
    VelodyneToPcl(pnts, cloud);
    pcl_conversions::toPCL(scan->packets[scan->packets.size() - 1].stamp, cloud.header.stamp);
    cloud.header.frame_id = scan->header.frame_id;
    return true;
}

template <class PointT>
bool MsgCloud2ToPCL(sensor_msgs::PointCloud2ConstPtr& point_cloud2, pcl::PointCloud<PointT>& cloud)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*point_cloud2, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.header.frame_id = point_cloud2->header.frame_id;
    pcl_conversions::toPCL(point_cloud2->header.stamp, cloud.header.stamp);
    return cloud.size() > 0;
}

template <class PointT>
bool MsgCloudToPCL(sensor_msgs::PointCloudConstPtr& point_cloud, pcl::PointCloud<PointT>& cloud)
{
    if (point_cloud != NULL)
    {
        for (int i = 0; i < point_cloud->points.size(); i++)
        {
            PointT p_target;
            geometry_msgs::Point32 p_src = point_cloud->points[i];
            p_target.x = p_src.x;
            p_target.y = p_src.y;
            p_target.z = p_src.z;
            cloud.push_back(p_target);
        }
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.header.frame_id = point_cloud->header.frame_id;
        pcl_conversions::toPCL(point_cloud->header.stamp, cloud.header.stamp);
        return cloud.size() > 0;
    }
}

template <class PointT>
bool LaserScanToPCL(sensor_msgs::LaserScanConstPtr& laser_scan, pcl::PointCloud<PointT>& cloud)
{
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 sens_cloud;
    projector.projectLaser(*laser_scan, sens_cloud);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(sens_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    pcl_conversions::toPCL(laser_scan->header.stamp, cloud.header.stamp);
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.header.frame_id = laser_scan->header.frame_id;

    return cloud.size() > 0;
}

template <class PointT>
void filter_fov_fun(pcl::PointCloud<PointT>& cloud,
                    pcl::PointCloud<PointT>& cloud_nofilter,
                    double hori_min,
                    double hori_max)
{
    for (int i = 0; i < cloud_nofilter.points.size(); ++i)
    {
        double ang = atan2(cloud_nofilter.points[i].y, cloud_nofilter.points[i].x);
        if (ang < hori_min || ang > hori_max)
            continue;
        cloud.points.push_back(cloud_nofilter.points[i]);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
}

template <class PointT>
void filter_range_fun(pcl::PointCloud<PointT>& cloud,
                      pcl::PointCloud<PointT>& cloud_nofilter,
                      double range_min,
                      double range_max)
{
    float min_dist = range_min * range_min;
    float max_dist = range_max * range_max;
    for (int i = 0; i < cloud_nofilter.points.size(); ++i)
    {
        float dist = cloud_nofilter.points[i].x * cloud_nofilter.points[i].x +
                     cloud_nofilter.points[i].y * cloud_nofilter.points[i].y +
                     cloud_nofilter.points[i].z * cloud_nofilter.points[i].z;
        if (dist < min_dist || dist > max_dist)
            continue;
        cloud.points.push_back(cloud_nofilter.points[i]);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.header.stamp = cloud_nofilter.header.stamp;
    cloud.header.frame_id = cloud_nofilter.header.frame_id;
}

template <class PointT>
void filter_height_angle(pcl::PointCloud<PointT>& cloud,
                         pcl::PointCloud<PointT>& cloud_nofilter,
                         double hori_min,
                         double hori_max,
                         double min_z,
                         double max_z)
{
    for (int i = 0; i < cloud_nofilter.points.size(); ++i)
    {
        double ang = atan2(cloud_nofilter.points[i].y, cloud_nofilter.points[i].x);
        if (ang < hori_min || ang > hori_max)
            continue;
        else if (cloud_nofilter.points[i].z < min_z || cloud_nofilter.points[i].z > max_z)
            continue;
        else
            cloud.points.push_back(cloud_nofilter.points[i]);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.header.frame_id = cloud_nofilter.header.frame_id;
    cloud.header.stamp = cloud_nofilter.header.stamp;
}

template <class PointT>
void filter_height_angle(
    pcl::PointCloud<PointT>& cloud, double hori_min, double hori_max, double min_z, double max_z)
{
    pcl::PointCloud<PointT> cloud_filtered;
    filter_height_angle(cloud_filtered, cloud, hori_min, hori_max, min_z, max_z);
    cloud.clear();
    cloud = cloud_filtered;
}

template <typename PointT>
void getMinMax3DPointCloud(const pcl::PointCloud<PointT>& cloud,
                           Eigen::Vector3d& minP,
                           Eigen::Vector3d& maxP)
{
    pcl::PointXYZL minPt, maxPt;
    pcl::getMinMax3D(cloud, minPt, maxPt);
    minP = PCLPointToEigen(minPt);
    maxP = PCLPointToEigen(maxPt);
}

template <class PointT> void AddVariance(pcl::PointCloud<PointT>& cloud, double varz)
{
    for (int i = 0; i < cloud.points.size(); i++)
        cloud.points[i].z += varz * (((double)rand()) / (double)RAND_MAX - 0.5);
}
template <class PointT> void PointCloudQueue<PointT>::Push(const pcl::PointCloud<PointT>& cloud)
{
    if (clouds.size() == max_size_)
    {
        // std::cout<<"queue size="<<max_size_<<" popping last element"<<std::endl;
        clouds.erase(clouds.begin());
    }

    clouds.push_back(cloud);
}
template <class PointT> void PointCloudQueue<PointT>::GetCloud(pcl::PointCloud<PointT>& cloud)
{
    if (clouds.size() == 0)
        return;

    cloud.clear();
    cloud.header.stamp = clouds[clouds.size() - 1].header.stamp;
    for (int i = 0; i < clouds.size(); i++)
    {
        cloud += clouds[i];
    }
    // std::cout<<"Got "<<cloud.size() <<" points"<<std::endl;
}
template <class PointT> void PointCloudQueue<PointT>::Save(const std::string& name_prefix)
{

    pcl::PointCloud<PointT> cloud_save;
    GetCloud(cloud_save);
    pcl::io::savePCDFileBinary(name_prefix + ndt_generic::toString(++cloud_set_) + ".pcd",
                               cloud_save);
}
} // namespace ndt_generic
#endif // PCL_UTILS_IMPL_H
