#include "graph_map_lamide/ndt_dl/point_curv3.h"


namespace perception_oru
{
namespace graph_map
{
/*bool similar(const float &x, const float &y){
  return(fabs(y-x)<vertical_lidar_tolerance);
}
*/

bool similar(const float& x, const float& y)
{
    return (fabs(y - x) < vertical_lidar_tolerance);
}

void ToXYZ(const std::vector<pcl::PointCloud<PointType>>& cloud,
           std::vector<pcl::PointCloud<pcl::PointXYZL>>& cloud_out)
{
    cloud_out.clear();
    cloud_out.resize(cloud.size());
    for (int i = 0; i < cloud.size(); i++)
    {
        for (int j = 0; j < cloud[i].size(); j++)
        {
            pcl::PointXYZL p;
            p.x = cloud[i][j].x;
            p.y = cloud[i][j].y;
            p.z = cloud[i][j].z;
            cloud_out[i].push_back(p);
        }
    }
}
void SegmentRings(const pcl::PointCloud<pcl::PointXYZL>& cloud,
                  std::vector<pcl::PointCloud<PointType>>& ring_clouds,
                  int& N_SCANS)
{

    ring_clouds.reserve(64);
    std::vector<double> rings;
    N_SCANS = 0;
    for (int i = 0; i < cloud.size(); i++)
    {
        PointType p;
        p.x = cloud[i].x;
        p.y = cloud[i].y;
        p.z = cloud[i].z;
        float y_angle = atan2(p.y, p.x) + M_PI; // 0-2*PI
        double r = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        float p_angle = asin(p.z / r) + M_PI; // 0-2*PI

        unsigned int ring_index = 0;
        bool ring_found = false;
        for (int i = 0; i < rings.size(); i++)
        {
            if (similar(p_angle, rings[i]))
            {
                ring_index = i;
                ring_found = true;
                break;
            }
        }
        if (!ring_found)
        {
            N_SCANS++;
            rings.push_back(p_angle);
        }
        p.ring = ring_index;
        ring_clouds[ring_index].push_back(p);
    }
}
void SegmentRings(const pcl::PointCloud<PointType>& cloud,
                  std::vector<pcl::PointCloud<PointType>>& ring_clouds,
                  int& N_SCANS)
{
    ring_clouds.reserve(64);
    N_SCANS = 0;
    for (int i = 0; i < cloud.size(); i++)
    {
        PointType p = cloud.points[i];
        ring_clouds[p.ring].push_back(p);
        if (p.ring > N_SCANS)
            N_SCANS = p.ring;
    }
}

void segmentPointCurvature3(const Eigen::Affine3d& Tsensor,
                            const pcl::PointCloud<PointType>& cloud,
                            std::vector<pcl::PointCloud<PointType>>& clouds_out)
{
    // Sort based on the ring number
    // Split the data based on each diode (ring)
    std::vector<pcl::PointCloud<PointType>> ring_clouds(64);
    int N_SCANS = 0;
    SegmentRings(cloud, ring_clouds, N_SCANS);
    ClassifyPoints(Tsensor, ring_clouds, N_SCANS, clouds_out);
}

void segmentPointCurvature3(const Eigen::Affine3d& Tsensor,
                            const pcl::PointCloud<PointType>& cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds_out)
{
    // Sort based on the ring number
    // Split the data based on each diode (ring)
    std::vector<pcl::PointCloud<PointType>> ring_clouds(64);
    std::vector<pcl::PointCloud<PointType>> segmented_clouds(64);
    int N_SCANS = 0;
    SegmentRings(cloud, ring_clouds, N_SCANS);
    ClassifyPoints(Tsensor, ring_clouds, N_SCANS, segmented_clouds);
    ToXYZ(segmented_clouds, clouds_out);
}
void segmentPointCurvature3(const Eigen::Affine3d& Tsensor,
                            const pcl::PointCloud<pcl::PointXYZL>& cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds_out)
{
    // Sort based on the ring number
    // Split the data based on each diode (ring)
    std::vector<pcl::PointCloud<PointType>> ring_clouds(64);
    std::vector<pcl::PointCloud<PointType>> segmented_clouds(64);
    int N_SCANS = 0;
    SegmentRings(cloud, ring_clouds, N_SCANS);

    ClassifyPoints(Tsensor, ring_clouds, N_SCANS, segmented_clouds);
    ToXYZ(segmented_clouds, clouds_out);
}
void ClassifyPoints(const Eigen::Affine3d& Tsensor,
                    std::vector<pcl::PointCloud<PointType>>& ring_clouds,
                    const int& N_SCANS,
                    std::vector<pcl::PointCloud<PointType>>& clouds_out)
{
    float cloudCurvature[160000];
    float range[160000];
    float incrAngle[160000];
    int cloudSortInd[160000];
    int cloudNeighborPicked[160000];
    int cloudLabel[160000];

    // Code from LOAM.
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    pcl::PointCloud<PointType> cloud;
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < ring_clouds.size(); i++)
    {
        *laserCloud += ring_clouds[i];
    }

    int cloudSize = laserCloud->size();

    int scanCount = -1;
    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x +
                      laserCloud->points[i - 3].x + laserCloud->points[i - 2].x +
                      laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                      laserCloud->points[i + 1].x + laserCloud->points[i + 2].x +
                      laserCloud->points[i + 3].x + laserCloud->points[i + 4].x +
                      laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y +
                      laserCloud->points[i - 3].y + laserCloud->points[i - 2].y +
                      laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                      laserCloud->points[i + 1].y + laserCloud->points[i + 2].y +
                      laserCloud->points[i + 3].y + laserCloud->points[i + 4].y +
                      laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z +
                      laserCloud->points[i - 3].z + laserCloud->points[i - 2].z +
                      laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                      laserCloud->points[i + 1].z + laserCloud->points[i + 2].z +
                      laserCloud->points[i + 3].z + laserCloud->points[i + 4].z +
                      laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;

        {
            float tmp_x = laserCloud->points[i].x - Tsensor.translation().x();
            float tmp_y = laserCloud->points[i].y - Tsensor.translation().y();
            float tmp_z = laserCloud->points[i].z - Tsensor.translation().z();

            range[i] = sqrt(tmp_x * tmp_x + tmp_y * tmp_y + tmp_z * tmp_z);

            float tmp2_x = laserCloud->points[i + 1].x - Tsensor.translation().x();
            float tmp2_y = laserCloud->points[i + 1].y - Tsensor.translation().y();
            float tmp2_z = laserCloud->points[i + 1].z - Tsensor.translation().z();

            range[i + 1] = sqrt(tmp2_x * tmp2_x + tmp2_y * tmp2_y + tmp2_z * tmp2_z);

            // incrAngle[i] = acos((tmp_x*tmp2_x + tmp_y*tmp2_y +
            // tmp_z*tmp2_z)/(range[i]*range[i+1]));
            incrAngle[i] =
                1 - (tmp_x * tmp2_x + tmp_y * tmp2_y + tmp_z * tmp2_z) /
                        (range[i] * range[i + 1]); // very rough approximation (if at all).
        }

        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;

        if (int(laserCloud->points[i].ring) != scanCount)
        {
            scanCount = int(laserCloud->points[i].ring);

            if (scanCount > 0 && scanCount < N_SCANS)
            {
                scanStartInd[scanCount] = i + 5;
                scanEndInd[scanCount - 1] = i - 5;
            }
        }
    }

    scanStartInd[0] = 5;
    scanEndInd.back() = cloudSize - 5;

    clouds_out.resize(3);
    for (size_t i = 0; i < clouds_out.size(); i++)
    {
        clouds_out[i].clear();
    }

    for (int i = 0; i < N_SCANS; i++)
    {

        typename pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(
            new pcl::PointCloud<PointType>);

        const int div = 6;
        for (int j = 0; j < div; j++)
        {

            int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / div;
            int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / div - 1;

            for (int k = ep; k >= sp; k--)
            {

                if (cloudCurvature[k] < 0.2)
                {

                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
                const float max_dist = 2;
                if (range[k] < range[k + 1] - max_dist && range[k - 1] < range[k + 1] - max_dist)
                {
                    {
                        clouds_out[1].push_back((laserCloud->points[k]));
                    }
                }
                else if (10 * incrAngle[k - 1] < incrAngle[k])
                { // To large difference indicates NaN value.
                    clouds_out[1].push_back((laserCloud->points[k]));
                }

                if (range[k] < range[k - 1] - max_dist && range[k + 1] < range[k - 1] - max_dist)
                {
                    {
                        clouds_out[2].push_back((laserCloud->points[k]));
                    }
                }
                else if (10 * incrAngle[k] < incrAngle[k - 1])
                {
                    clouds_out[2].push_back((laserCloud->points[k]));
                }
            }
        }
        clouds_out[0] += *surfPointsLessFlatScan;
    }
}

} // namespace graph_map
} // namespace perception_oru
