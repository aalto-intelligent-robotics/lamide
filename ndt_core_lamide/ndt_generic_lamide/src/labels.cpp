#include <ndt_generic_lamide/labels.h>
#include <algorithm>


namespace ndt_generic
{

bool isStatic(int label)
{
    return (std::find(static_labels_.begin(), static_labels_.end(), label) != static_labels_.end());
}

bool isDynamic(int label)
{
    return (std::find(dynamic_labels_.begin(), dynamic_labels_.end(), label) !=
            dynamic_labels_.end());
}

bool isSemiStatic(int label)
{
    return (std::find(semistatic_labels_.begin(), semistatic_labels_.end(), label) !=
            semistatic_labels_.end());
}

void filterNonStaticPoints(pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>> filtered;
    filtered.resize(cloud.points.size());
    int idx = 0;
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        if (std::find(static_labels_.begin(), static_labels_.end(), cloud.points[i].label) !=
            static_labels_.end())
        {
            filtered[idx] = cloud.points[i];
            idx++;
        }
    }
    // std::cout << "original size: " << cloud.points.size() << std::endl;
    // std::cout << "filtered size: " << idx << std::endl;
    filtered.resize(idx + 1);
    cloud.points.swap(filtered);
}

void filterDynamicPoints(pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    std::vector<pcl::PointXYZL, Eigen::aligned_allocator<pcl::PointXYZL>> filtered;
    filtered.resize(cloud.points.size());
    int idx = 0;
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        if (std::find(dynamic_labels_.begin(), dynamic_labels_.end(), cloud.points[i].label) ==
            dynamic_labels_.end())
        {
            filtered[idx] = cloud.points[i];
            idx++;
        }
    }

    // std::cout << "original size: " << cloud.points.size() << std::endl;
    // std::cout << "filtered size: " << idx << std::endl;
    filtered.resize(idx + 1);
    cloud.points.swap(filtered);
}

} // namespace ndt_generic
