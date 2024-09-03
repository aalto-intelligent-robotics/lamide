#include <ndt_generic_lamide/pcl_utils.h>

namespace ndt_generic
{

void convertPointCloud(const pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& pnts,
                       pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    for (size_t i = 0; i < pnts.size(); i++)
    {
        pcl::PointXYZL p;
        p.x = pnts.points[i].x;
        p.y = pnts.points[i].y;
        p.z = pnts.points[i].z;
        cloud.push_back(p);
    }
    cloud.header.frame_id = pnts.header.frame_id;
    cloud.header.stamp = pnts.header.stamp;
    cloud.width = cloud.size();
    cloud.height = 1;
}

void convertPointCloud(const pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& pnts,
                       pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud)
{
    cloud = pnts;
}

void VelodyneToPcl(velodyne_pointcloud_oru::PointcloudXYZIR& pnts,
                   pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud)
{

    sensor_msgs::PointCloud2 cld = pnts.finishCloud();
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cld, "x"), iter_y(cld, "y"),
        iter_z(cld, "z"), iter_i(cld, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(cld, "ring");
    pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR> cloud_tmp;
    // cloud_tmp.resize(cld.width*cld.height*cld.point_step);
    int idx = 0;
    while (iter_x != iter_x.end())
    {
        velodyne_pointcloud_oru::PointXYZIR p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.intensity = *iter_i;
        p.ring = *iter_r;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_i;
        ++iter_r;
        cloud.push_back(p);
    }
    cloud.width = cloud.size();
    cloud.height = 1;
}

void VelodyneToPcl(velodyne_pointcloud_oru::PointcloudXYZIR& pnts,
                   pcl::PointCloud<pcl::PointXYZL>& cloud)
{

    sensor_msgs::PointCloud2 cld = pnts.finishCloud();
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cld, "x"), iter_y(cld, "y"),
        iter_z(cld, "z");
    pcl::PointCloud<pcl::PointXYZL> cloud_tmp;
    // cloud_tmp.resize(cld.width*cld.height*cld.point_step);
    int idx = 0;
    while (iter_x != iter_x.end())
    {
        pcl::PointXYZL p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        cloud.push_back(p);
    }
}

// Point cloud processing utils...
void computeDirectionsAndRangesFromPointCloud(const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                              const Eigen::Vector3d& origin,
                                              std::vector<Eigen::Vector3d>& dirs,
                                              std::vector<double>& ranges)
{
    ranges.reserve(cloud.size());
    dirs.reserve(cloud.size());
    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = cloud.points.begin();

    while (it != cloud.points.end())
    {
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }

        Eigen::Vector3d diff;
        diff << it->x - origin(0), it->y - origin(1), it->z - origin(2);
        ranges.push_back(diff.norm());
        diff.normalize();
        dirs.push_back(diff);
        it++;
    }
}
pcl::PointXYZL eigenToPCLPoint(const Eigen::Vector3d& pt)
{
    pcl::PointXYZL p;
    p.x = pt[0];
    p.y = pt[1];
    p.z = pt[2];
    return p;
}

Eigen::Vector3d PCLPointToEigen(const pcl::PointXYZL& pt)
{
    return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

} // namespace ndt_generic
