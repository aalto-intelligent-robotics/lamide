#include "graph_map_lamide/ndt/ndt_map_type.h"

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTMapType)
namespace perception_oru
{
namespace graph_map
{
using namespace std;

using namespace perception_oru;

NDTMapType::NDTMapType(MapParamPtr paramptr)
    : MapType(paramptr)
{
    NDTMapParamPtr param = boost::dynamic_pointer_cast<NDTMapParam>(paramptr); // Should not be NULL
    if (param != NULL)
    {
        resolution_ = param->resolution_;
        map_ = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution_));
        map_->initialize(0.0, 0.0, 0.0, param->sizex_, param->sizey_, param->sizez_,
                         param->positive_update_static_, param->negative_update_static_, param->eta_static_,
                         param->positive_update_dynamic_, param->negative_update_dynamic_, param->eta_dynamic_, param->w_own_, param->w_cluster_);
        max_occupancy_ = param->max_occupancy_;
        sensor_noise_ = param->sensor_noise_;
        maxNumPoints_ = param->maxNumPoints_;
    }
    else
    {
        cerr << "Cannot create instance of NDTmap" << std::endl;
    }
}

NDTMapType::~NDTMapType()
{
    delete map_;
}

// update map, cloud is the scan, Tsensor is the pose where the scan was aquired.
void NDTMapType::update(const Eigen::Affine3d& Tsensor,
                        pcl::PointCloud<pcl::PointXYZL>& cloud,
                        bool simple,
                        bool cluster)
{

    if (initialized_ && cloud.size() > 0)
    {
        Eigen::Vector3d localMapSize(2 * max_range_, 2 * max_range_, sizez_);
        if (!simple)
        {
            if(cluster)
            {
                map_->addPointCloudClusterUpdate(Tsensor.translation(), cloud, localMapSize,
                maxNumPoints_, max_occupancy_, sizez_, sensor_noise_);
            }
            else
            {
                map_->addPointCloudMeanUpdate(Tsensor.translation(), cloud, localMapSize, maxNumPoints_, max_occupancy_,
                                            sizez_, sensor_noise_);

            }
        }
        else
        {
            map_->addPointCloudSimple(cloud, sizez_);
            map_->computeNDTCells();
        }
    }
    else if (!initialized_ && cloud.size() > 0)
    {
        InitializeMap(Tsensor, cloud, simple, cluster);
        initialized_ = true;
    }
}
void NDTMapType::PlotScan(pcl::PointCloud<pcl::PointXYZL>& cloud,
                          Eigen::Affine3d& T,
                          const std::string& frame_id)
{
    perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution_));
    ndlocal.guessSize(0, 0, 0, 130, 130, 20);
    ndlocal.loadPointCloud(cloud, 130);
    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
    ndt_map_lamide::NDTMapRGBMsg msg;
    std::vector<NDTCell*> cells = ndlocal.pseudoTransformNDT(T);
    pcl::PointCloud<pcl::PointXYZL> cloud2 = cloud;
    transformPointCloudInPlace(T, cloud2);
    cloud2.header.frame_id = frame_id;
    toMessage(cells, msg, frame_id);
    ndt_pub.publish(msg);
    ros::Time t = ros::Time::now();
    for (NDTCell* p : cells)
    {
        delete p;
    }
    pcl_conversions::toPCL(t, cloud.header.stamp);

    cloud_pub.publish(cloud);
}
void NDTMapType::update(const Eigen::Affine3d& Tsensor,
                        std::vector<pcl::PointCloud<pcl::PointXYZL>>& clouds,
                        bool simple,
                        bool cluster)
{ // update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

    if (clouds.empty())
    {
        return;
    }
    update(Tsensor, clouds[0], simple, cluster);
}

void NDTMapType::InitializeMap(const Eigen::Affine3d& Tsensor,
                               pcl::PointCloud<pcl::PointXYZL>& cloud,
                               bool simple,
                               bool cluster)
{
    // cout << "initialize map" << endl;
    if (!simple)
    {
        map_->addPointCloud(Tsensor.translation(), cloud, 0.1, 100.0, 0.1);
        map_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(),
                              0.1);
    }
    else
    {
        map_->addPointCloudSimple(cloud, sizez_);
        map_->computeNDTCells();
    }
    if(cluster)
    {
        map_->cluster(true, true, false);
    }
}

void NDTMapType::GetCloud(pcl::PointCloud<pcl::PointXYZL>& cloud, bool downsampled)
{
    if (downsampled)
    {
        cloud = downsampled_cloud_;
    }
    if (ndt_filtered_cloud_)
    {
        NDTfilterPointcloud(cloud, 1.0);
    }
    else if (ndt_cell_cloud_)
    {
        NDTMap* ndt_map = GetNDTMap();
        std::vector<NDTCell*> cells = ndt_map->getAllCells();
        for (int i = 0; i < cells.size(); i++)
        {
            Eigen::Vector3d pt = cells[i]->getMean();
            pcl::PointXYZL pcl_point;
            pcl_point.x = pt(0);
            pcl_point.y = pt(1);
            pcl_point.z = pt(2);
            pcl_point.label = cells[i]->getLabel();
            cloud.push_back(pcl_point);
        }
    }
    else
    {
        MapType::GetCloud(cloud);
    }
}

void NDTMapType::NDTfilterPointcloud(pcl::PointCloud<pcl::PointXYZL>& cloud, double sigma_distance)
{
    NDTCell* cell = new NDTCell();
    int pnts_added = 0, pnts_with_cell = 0, points = 0;

    for (int i = 0; i < input_cloud_.size(); i++)
    {
        for (int j = 0; j < input_cloud_[i].size(); j++)
        {
            points++;
            if (map_->getCellAtPoint(input_cloud_[i][j], cell) && cell->hasGaussian_)
            {
                pnts_with_cell++;
                Eigen::Vector3d p, u, x;
                u = cell->getMean();
                p << input_cloud_[i][j].x, input_cloud_[i][j].y, input_cloud_[i][j].z;
                x = p - u;
                double mahala_d = sqrt(x.transpose() * cell->getInverseCov() * x);
                if (mahala_d < sigma_distance)
                {
                    cloud.push_back(input_cloud_[i][j]);
                    pnts_added++;
                }
            }
        }
    }
    cout << "Total points: " << points << "pnts_with_cell:" << pnts_with_cell
         << ", pnts_added:" << pnts_added << endl;
}

double NDTMapType::Score(MapTypePtr& source, const Affine3d& T)
{
    double l2_score = 0;
    NDTMap* target_ndt = this->GetNDTMap();
    if (source == NULL || target_ndt == NULL)
        return 0;
    NDTMap* source_ndt = boost::dynamic_pointer_cast<NDTMapType>(source)->GetNDTMap();

    std::vector<NDTCell*> source_cells = source_ndt->pseudoTransformNDT(T);
    if (source_cells.size() != 0)
    {
        l2_score = L2_Score(target_ndt, source_cells);
    }
    for (NDTCell* p : source_cells)
    {
        delete p;
    }
    return l2_score;
}
double NDTMapType::Overlap(MapTypePtr& source, const Affine3d& T)
{
    if (source == NULL || map_ == NULL)
        return 0;
    NDTCell c;
    unsigned int overlapping_cells = 0;
    std::vector<NDTCell*> cells =
        boost::dynamic_pointer_cast<NDTMapType>(source)->GetNDTMap()->pseudoTransformNDT(T);
    //NDTCell* cell = new NDTCell();
    NDTCell* cell = NULL;
    for (int i = 0; i < cells.size(); i++)
    {
        Eigen::Vector3d cell_mean = cells[i]->getMean();
        pcl::PointXYZL cell_mean_point;
        cell_mean_point.x = cell_mean(0);
        cell_mean_point.y = cell_mean(1);
        cell_mean_point.z = cell_mean(2);
        map_->getCellForPoint(cell_mean_point, cell, true);
        if (cell != NULL)
        {
            overlapping_cells++;
        }
    }
    //delete cell;
    for (NDTCell* p : cells)
    {
        delete p;
    }
    return overlapping_cells;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr NDTMapType::DownSampleCloud()
{
    NDTfilterPointcloud(downsampled_cloud_, 1);
    return NULL;
}

bool NDTMapType::CompoundMapsByRadius(MapTypePtr target,
                                      const Affine3d& T_source,
                                      const Affine3d& T_target,
                                      double radius)
{

    Affine3d Tdiff = Affine3d::Identity();
    Tdiff = T_source.inverse() * T_target;
    pcl::PointXYZL center_pcl;
    center_pcl.x = Tdiff.translation()(0);
    center_pcl.y = Tdiff.translation()(1);
    center_pcl.z = Tdiff.translation()(2);
    if (NDTMapPtr targetPtr = boost::dynamic_pointer_cast<NDTMapType>(target))
    {
        if (resolution_ != targetPtr->resolution_) // checking if source and target have same
                                                   // resolution, they shoould have.
            return false;

        if (radius == -1) // if radius is not defined, match rcenter_pcladius to size of new map
            radius = targetPtr->sizex_ < targetPtr->sizey_ ? targetPtr->sizex_ / 2
                                                           : targetPtr->sizey_ / 2;

        int neighboors = radius / resolution_;
        std::vector<NDTCell*> cells = map_->getCellsForPoint(center_pcl, neighboors, true);
        Tdiff = T_source.inverse() * T_target;
        for (int i = 0; i < cells.size(); i++)
        {
            Eigen::Matrix3d cov = Tdiff.inverse().linear() * cells[i]->getCov() * Tdiff.linear();
            Eigen::Vector3d mean = Tdiff.inverse() * cells[i]->getMean();
            targetPtr->GetNDTMap()->addDistributionToCell(cov, mean, 3);
            pcl::PointXYZL p;
            p.x = mean.x();
            p.y = mean.y();
            p.z = mean.z();
            NDTCell* cell;
            bool foundCell = targetPtr->GetNDTMap()->getCellAtPoint(p, cell);
            if(foundCell)
            {
                int l = cells[i]->getLabel();
                int w = cells[i]->getLabelWeight();
                cell->setLabel(l, w);

                // int idx = cells[i]->getClusterId();
                // cell->setClusterId(idx);
            }
        }
    }
}

std::string NDTMapType::ToString()
{
    stringstream ss;
    ss << MapType::ToString() << "NDT Map Type:" << endl;
    ss << "resolution:" << resolution_ << endl;
    ss << "resolution local factor:" << resolution_local_factor_ << endl;
    ss << "nr active cells:" << map_->numberOfActiveCells() << endl;
    return ss.str();
}

} // namespace graph_map
} // namespace perception_oru
