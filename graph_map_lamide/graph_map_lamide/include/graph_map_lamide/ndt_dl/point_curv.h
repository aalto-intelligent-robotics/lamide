#ifndef NDTDL_POINT_CURV_H
#define NDTDL_POINT_CURV_H

#include <angles/angles.h>
#include <ndt_generic_lamide/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>


namespace perception_oru
{
namespace graph_map
{

typedef velodyne_pointcloud_oru::PointXYZIR PointType;

double getDoubleTime()
{
    struct timeval time;
    gettimeofday(&time, NULL);
    return time.tv_sec + time.tv_usec * 1e-6;
}

void segmentPointCurvature(const pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                           pcl::PointCloud<pcl::PointXYZL>& cornerPointsSharp,
                           pcl::PointCloud<pcl::PointXYZL>& cornerPointsLessSharp,
                           pcl::PointCloud<pcl::PointXYZL>& surfPointsFlat,
                           pcl::PointCloud<pcl::PointXYZL>& surfPointsLessFlat)
{

    double t0 = getDoubleTime();

    float cloudCurvature[160000];
    int cloudSortInd[160000];
    int cloudNeighborPicked[160000];
    int cloudLabel[160000];

    // Sort based on the ring number
    // Split the data based on each diode (ring)
    std::vector<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>> ring_clouds(64);
    ring_clouds.reserve(64);
    int N_SCANS = 0;
    for (int i = 0; i < cloud.size(); i++)
    {
        velodyne_pointcloud_oru::PointXYZIR p = cloud.points[i];
        ring_clouds[p.ring].push_back(p);
        if (p.ring > N_SCANS)
            N_SCANS = p.ring;
    }

    // Code from LOAM.
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < ring_clouds.size(); i++)
    {
        *laserCloud += ring_clouds[i];
    }

    double t12 = 0;
    double t23 = 0;
    double t34 = 0;
    double t45 = 0;

    double t_12 = 0;
    double t_23 = 0;
    double t_34 = 0;
    double t_45 = 0;

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

    for (int i = 5; i < cloudSize - 6; i++)
    {
        float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
        float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
        float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.1)
        {

            float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                                laserCloud->points[i].y * laserCloud->points[i].y +
                                laserCloud->points[i].z * laserCloud->points[i].z);

            float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                                laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                                laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

            if (depth1 > depth2)
            {
                diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
                diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
                diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
                {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            }
            else
            {
                diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
        float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
        float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        float dis = laserCloud->points[i].x * laserCloud->points[i].x +
                    laserCloud->points[i].y * laserCloud->points[i].y +
                    laserCloud->points[i].z * laserCloud->points[i].z;

        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis)
        {
            cloudNeighborPicked[i] = 1;
        }
    }

    cornerPointsSharp.clear();
    cornerPointsLessSharp.clear();
    surfPointsFlat.clear();
    surfPointsLessFlat.clear();

    double t1 = getDoubleTime();

    for (int i = 0; i < N_SCANS; i++)
    {

        t1 = getDoubleTime();

        double t2 = getDoubleTime();

        t12 += (t2 - t1);

        // pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        pcl::PointCloud<pcl::PointXYZL>::Ptr surfPointsLessFlatScan(
            new pcl::PointCloud<pcl::PointXYZL>);

        const int div = 6;
        for (int j = 0; j < div; j++)
        {

            t2 = getDoubleTime();

            double t_1 = getDoubleTime();
            int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / div;
            int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / div - 1;

            for (int k = sp + 1; k <= ep; k++)
            {
                for (int l = k; l >= sp + 1; l--)
                {
                    if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]])
                    {
                        int temp = cloudSortInd[l - 1];
                        cloudSortInd[l - 1] = cloudSortInd[l];
                        cloudSortInd[l] = temp;
                    }
                }
            }

            double t_2 = getDoubleTime();
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(pcl::PointXYZL(laserCloud->points[ind].x,
                                                                  laserCloud->points[ind].y,
                                                                  laserCloud->points[ind].z));
                        cornerPointsLessSharp.push_back(pcl::PointXYZL(laserCloud->points[ind].x,
                                                                      laserCloud->points[ind].y,
                                                                      laserCloud->points[ind].z));
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(pcl::PointXYZL(laserCloud->points[ind].x,
                                                                      laserCloud->points[ind].y,
                                                                      laserCloud->points[ind].z));
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX =
                            laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY =
                            laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ =
                            laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX =
                            laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY =
                            laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ =
                            laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            double t_3 = getDoubleTime();
            t_12 += t_2 - t_1;
            t_23 += t_3 - t_2;

            double t3 = getDoubleTime();

            t23 += (t3 - t2);

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (ind < 5)
                {
                    // Henrik: this happens occationally -> TODO find out why. Commonly the idx ==
                    // 0.
                    break;
                }

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(pcl::PointXYZL(laserCloud->points[ind].x,
                                                           laserCloud->points[ind].y,
                                                           laserCloud->points[ind].z));

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX =
                            laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY =
                            laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ =
                            laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX =
                            laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY =
                            laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ =
                            laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            double t4 = getDoubleTime();
            t34 += (t4 - t3);
            // surfPointsLessFlatScan->clear();
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
#if 1
                    surfPointsLessFlatScan->push_back(pcl::PointXYZL(
                        laserCloud->points[k].x, laserCloud->points[k].y, laserCloud->points[k].z));
#else
                    surfPointsLessFlat.push_back(pcl::PointXYZL(
                        laserCloud->points[k].x, laserCloud->points[k].y, laserCloud->points[k].z));
#endif
                }
            }
            double t5 = getDoubleTime();
            t45 += (t5 - t4);
        }

#if 1
        pcl::PointCloud<pcl::PointXYZL> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZL> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        surfPointsLessFlat += surfPointsLessFlatScanDS;

#else
        surfPointsLessFlat += *surfPointsLessFlatScan;
#endif
    }
    // std::cout << "t12 : " << t12 << " t23 : " << t23 << " t34 : " << t34 << " t45 : " << t45 <<
    // std::endl; std::cout << "t_12 : " << t_12 << "t_23 : " << t_23 << std::endl;
}

} // namespace graph_map
} // namespace perception_oru

#endif
