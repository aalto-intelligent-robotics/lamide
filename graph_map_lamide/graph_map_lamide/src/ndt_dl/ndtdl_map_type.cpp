#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"

#include <boost/serialization/export.hpp>
#include <graph_map_lamide/ndt_dl/point_curv3.h>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTDLMapType)

namespace perception_oru
{
namespace graph_map
{

NDTDLMapType::NDTDLMapType(MapParamPtr paramptr)
    : MapType(paramptr)
{
    NDTDLMapParamPtr param =
        boost::dynamic_pointer_cast<NDTDLMapParam>(paramptr); // Should not be NULL
    if (param != NULL)
    {
        resolutions_ = param->resolutions_;
        for (size_t i = 0; i < resolutions_.size(); i++)
        {
            perception_oru::NDTMap* map =
                new perception_oru::NDTMap(new perception_oru::LazyGrid(resolutions_[i]));
            map->initialize(0.0, 0.0, 0.0, param->sizex_, param->sizey_, param->sizez_, 0.6,
                            0.4, 0.2, 0.7, 0.3, 0.8, 1.0, 0.5);
            maps_.push_back(map);
        }
        cout << "created ndtdlmap" << endl;
    }
    else
        cerr << "templateMapType: Cannot create instance for \"templateMapType\"" << std::endl;
}
NDTDLMapType::~NDTDLMapType()
{
}

void NDTDLMapType::update(const Eigen::Affine3d& Tsensor,
                          pcl::PointCloud<pcl::PointXYZL>& cloud,
                          bool simple,
                          bool cluster)
{ // update map, cloud is the scan, Tsensor is the pose where the scan was aquired.
    (void)cluster;
    cout << "The NDT-DL cannot be updated with a single semantic point class" << endl;
    if (initialized_)
    {
        // Initialize map
    }
    else
    {
        // Update map
        initialized_ = true;
    }
}

void NDTDLMapType::update(const Eigen::Affine3d& Tsensor,
                          std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                          bool simple,
                          bool cluster)
{ // update map, cloud is the scan, Tsensor is the pose where the scan was aquired.
    (void)cluster;
    if (clouds.empty())
    {
        return;
    }

    if (initialized_ && enable_mapping_)
    {
        Eigen::Vector3d localMapSize(max_range_, max_range_, sizez_);
        for (size_t i = 0; i < clouds.size(); i++)
        {
            this->maps_[i]->addPointCloudMeanUpdate(Tsensor.translation(), clouds[i], localMapSize,
                                                    1e5, 25, 2 * sizez_, 0.06);
        }
    }
    else if (!initialized_)
    {
        InitializeMap(Tsensor, clouds);
        initialized_ = true;
    }
}

void NDTDLMapType::InitializeMap(const Eigen::Affine3d& Tsensor,
                                 std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds)
{
    cout << "initialize map" << endl;
    for (size_t i = 0; i < clouds.size(); i++)
    {
        maps_[i]->addPointCloud(Tsensor.translation(), clouds[i], 0.1, 100.0, 0.1);
        maps_[i]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(),
                                  0.1);
    }
}

bool NDTDLMapType::CompoundMapsByRadius(MapTypePtr target,
                                        const Affine3d& T_source,
                                        const Affine3d& T_target,
                                        double radius)
{
    if (NDTDLMapPtr targetPtr = boost::dynamic_pointer_cast<NDTDLMapType>(target))
    {

        Affine3d Tdiff = Affine3d::Identity();
        Tdiff = T_source.inverse() * T_target;
        pcl::PointXYZL center_pcl;
        center_pcl.x = Tdiff.translation()(0);
        center_pcl.y = Tdiff.translation()(1);
        center_pcl.z = Tdiff.translation()(2);

        if (radius == -1) // if radius is not defined, match rcenter_pcladius to size of new map
            radius = targetPtr->sizex_ < targetPtr->sizey_ ? targetPtr->sizex_ / 2
                                                           : targetPtr->sizey_ / 2;
        for (int i = 0; i < maps_.size(); i++)
        {
            int neighboors = radius / resolutions_[i];
            std::vector<NDTCell*> cells = maps_[i]->getCellsForPoint(center_pcl, neighboors, true);
            Tdiff = T_source.inverse() * T_target;
            for (int j = 0; j < cells.size(); j++)
            {
                Eigen::Matrix3d cov =
                    Tdiff.inverse().linear() * cells[j]->getCov() * Tdiff.linear();
                Eigen::Vector3d mean = Tdiff.inverse() * cells[j]->getMean();
                targetPtr->GetNDTMaps()[i]->addDistributionToCell(cov, mean, 3);
            }
        }
    }
}

} // namespace graph_map
} // namespace perception_oru
