#ifndef NDT_CONVERSIONS_HH
#define NDT_CONVERSIONS_HH

#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/NDTMapRGBMsg.h>
#include <ndt_map_lamide/NDTCellMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include "colormap.h"
#include <ndt_generic_lamide/labels.h>

namespace perception_oru
{

inline bool toMessage(const std::vector<perception_oru::NDTCell*>& map_vector,
                      ndt_map_lamide::NDTMapRGBMsg& msg,
                      const std::string& frame_name)
{

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_name; // is it in *map?

    for (int cell_idx = 0; cell_idx < map_vector.size(); cell_idx++)
    {
        if (map_vector[cell_idx] != NULL)
        { // we send intialized cells

            ndt_map_lamide::NDTCellRGBMsg cell;
            cell.hasGaussian_ = map_vector[cell_idx]->hasGaussian_;

            cell.center_x = map_vector[cell_idx]->getCenter().x;
            cell.center_y = map_vector[cell_idx]->getCenter().y;
            cell.center_z = map_vector[cell_idx]->getCenter().z;

            if (map_vector[cell_idx]->hasGaussian_)
            { // we only send a cell with gaussian

                Eigen::Vector3d means = map_vector[cell_idx]->getMean();
                cell.mean_x = means(0);
                cell.mean_y = means(1);
                cell.mean_z = means(2);
                cell.occupancy = map_vector[cell_idx]->getOccupancyRescaled();
                Eigen::Matrix3d cov = map_vector[cell_idx]->getCov();
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        cell.cov_matrix.push_back(cov(i, j));
                    }
                }
                cell.N = map_vector[cell_idx]->getN();
            }
            msg.cells.push_back(cell);
        }
    }
    return true;
}

inline void mapClusterToRGB(int cluster, float& r, float& g, float& b)
{
    if (cluster >= 0)
    {
        int idx = cluster % 2197;
        r = RND_COLORMAP_2197[idx][0];
        g = RND_COLORMAP_2197[idx][1];
        b = RND_COLORMAP_2197[idx][2];
    }
    else
    {
        r = 0.2;
        g = 0.8;
        b = 1;
    }
}

inline void getColor(int color, NDTCell* cell, float& r, float& g, float& b)
{
    bool single_color = color == 0;
    bool color_by_cluster = color == 2;
    bool color_by_update = color == 3;
    bool color_by_occupancy = color == 4;
    bool color_by_membership = color == 5;
    bool color_by_dynamics = color == 6;
    bool color_by_comparison = color == 100;

    if (color_by_cluster)
    {
        int cluster_id = cell->getClusterId();
        mapClusterToRGB(cluster_id, r, g, b);
    }
    else if (color_by_update)
    {
        bool up = cell->viz_updated_;
        bool ac = cell->viz_accessed_;
        bool cr = cell->viz_created_;
        float oc = cell->viz_occ_update_;
        if (up || ac)
        {
            if (5 < oc)
            {
                r = 0;
                g = 1;
                b = 0;
            }
            else if (1 < oc && oc <= 5)
            {
                r = 0;
                g = 0.75;
                b = 0;
            }
            else if (0 < oc && oc <= 1)
            {
                r = 0;
                g = 0.75;
                b = 0.5;
            }
            else if (oc == 0)
            {
                r = 0;
                g = 0.35;
                b = 0.7;
            }
            else if (-1 < oc && oc < 0)
            {
                r = 1;
                g = 1;
                b = 0;
            }
            else if (-5 < oc && oc <= -1)
            {
                r = 1;
                g = 0.6;
                b = 0;
            }
            else
            {
                r = 1;
                g = 0.3;
                b = 0;
            }
        }
        else if (cr)
        {
            r = 1;
            g = 0.65;
            b = 0.9;
        }
        else
        {
            r = 0.35;
            g = 0.70;
            b = 1;
        }
    }
    else if (single_color)
    {
        r = 0.8;
        g = 0.2;
        b = 0.8;
    }
    else if (color_by_occupancy)
    {
        float oc = cell->getOccupancy();
        r = std::min(std::max(2.0 - (2.0 * oc / 100.0), 0.0), 1.0);
        g = std::min(std::max((2.0 * oc / 100.0), 0.0), 1.0);
        b = 0.0;
    }
    else if (color_by_membership)
    {
        double mem = cell->getClusterMembership();
        r = std::min(std::max(2.0 - (2.0 * mem), 0.0), 1.0);
        g = std::min(std::max((2.0 * mem), 0.0), 1.0);
        b = 0.0;
    }
    else if (color_by_dynamics)
    {
        int l = cell->getLabel();
        if(ndt_generic::isStatic(l))
        {
            r = 0.0;
            g = 0.5;
            b = 1.0;
        }
        else if (ndt_generic::isSemiStatic(l))
        {
            r = 1.0;
            g = 1.0;
            b = 0.0;
        }
        else if (ndt_generic::isDynamic(l))
        {
            r = 1.0;
            g = 0.0;
            b = 0.0;
        }
        else
        {
            r = 1.0;
            g = 1.0;
            b = 1.0;
        }
    }
    else if (color_by_comparison)
    {
        NDTCell::ComparisonState state = cell->getComparisonStatus();
        switch (state)
        {
            case NDTCell::ComparisonState::OTHER_MISSING: {
                r = 0;
                g = 0.5;
                b = 0.5;
                break;
            }
            case NDTCell::ComparisonState::OTHER_NO_DISTRIBUTION: {
                r = 0;
                g = 0.5;
                b = 1;
                break;
            }
            case NDTCell::ComparisonState::THIS_MISSING: {
                r = 0.5;
                g = 0;
                b = 0.5;
                break;
            }
            case NDTCell::ComparisonState::THIS_NO_DISTRIBUTION: {
                r = 0.5;
                g = 0;
                b = 1;
                break;
            }
            case NDTCell::ComparisonState::MATCH: {
                double dist = cell->getComparisonDistance();
                r = std::min(std::max(2.0 - (2.0 * dist), 0.0), 1.0);
                g = std::min(std::max((2.0 * dist), 0.0), 1.0);
                b = 0.0;
                break;
            }
            case NDTCell::ComparisonState::NOT_SET: {
                r = 0.4;
                g = 0.4;
                b = 0.4;
                break;
            }
            default: {
                r = 0.4;
                g = 0.4;
                b = 0.4;
                break;
            }
        }
    }
    else
    {
        cell->getRGB(r, g, b);
    }
}

inline bool toRGBMessage(const std::vector<perception_oru::NDTCell*>& map_vector,
                         ndt_map_lamide::NDTMapRGBMsg& msg,
                         const std::string& frame_name,
                         int color = 1)
{

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_name; // is it in *map?

    for (int cell_idx = 0; cell_idx < map_vector.size(); cell_idx++)
    {
        if (map_vector[cell_idx] != NULL)
        { // we send intialized cells

            ndt_map_lamide::NDTCellRGBMsg cell;
            cell.hasGaussian_ = map_vector[cell_idx]->hasGaussian_;

            cell.center_x = map_vector[cell_idx]->getCenter().x;
            cell.center_y = map_vector[cell_idx]->getCenter().y;
            cell.center_z = map_vector[cell_idx]->getCenter().z;

            float r, g, b;
            getColor(color, map_vector[cell_idx], r, g, b);
            cell.red = r;
            cell.green = g;
            cell.blue = b;
            cell.label = map_vector[cell_idx]->getLabel();
            cell.label_weight = map_vector[cell_idx]->getLabelWeight();
            cell.label_time = map_vector[cell_idx]->getLabelTime();

            if (map_vector[cell_idx]->hasGaussian_)
            { // we only send a cell with gaussian

                Eigen::Vector3d means = map_vector[cell_idx]->getMean();
                cell.mean_x = means(0);
                cell.mean_y = means(1);
                cell.mean_z = means(2);
                cell.occupancy = map_vector[cell_idx]->getOccupancyRescaled();
                Eigen::Matrix3d cov = map_vector[cell_idx]->getCov();
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        cell.cov_matrix.push_back(cov(i, j));
                    }
                }
                cell.N = map_vector[cell_idx]->getN();
            }
            msg.cells.push_back(cell);
        }
    }
    return true;
}

/**
 *
 * \brief Message building fucntion
 * \details Converts an object of type NDTMap into NDTMapMsg
 * message. Message contain all the data strored in the object.
 * @param[in] map Pointer to NDTMap object.
 * @param[out] msg formated message
 * @param[in] frame_name name of the coordination frame for the transformed map
 *
 */
inline bool toMessage(NDTMap* map, ndt_map_lamide::NDTMapRGBMsg& msg, std::string frame_name)
{
    std::vector<perception_oru::NDTCell*> map_vector = map->getAllInitializedCells();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_name; // is it in *map?
    if (!map->getGridSizeInMeters(msg.x_size, msg.y_size, msg.z_size))
    {
        ROS_ERROR("NO GRID SIZE");
        return false;
    }
    if (!map->getCentroid(msg.x_cen, msg.y_cen, msg.z_cen))
    {
        ROS_ERROR("NO GRID CENTER");
        return false;
    }
    if (!map->getCellSizeInMeters(msg.x_cell_size, msg.y_cell_size, msg.z_cell_size))
    {
        ROS_ERROR("NO CELL SIZE");
        return false;
    }
    for (int cell_idx = 0; cell_idx < map_vector.size(); cell_idx++)
    {
        if (map_vector[cell_idx] != NULL)
        { // we send intialized cells

            ndt_map_lamide::NDTCellRGBMsg cell;
            cell.hasGaussian_ = map_vector[cell_idx]->hasGaussian_;

            cell.center_x = map_vector[cell_idx]->getCenter().x;
            cell.center_y = map_vector[cell_idx]->getCenter().y;
            cell.center_z = map_vector[cell_idx]->getCenter().z;

            cell.occupancy = map_vector[cell_idx]->getOccupancyRescaled();
            cell.occupancy_raw = map_vector[cell_idx]->getOccupancy();

            if (map_vector[cell_idx]->hasGaussian_)
            { // we only send a cell with gaussian

                Eigen::Vector3d means = map_vector[cell_idx]->getMean();
                cell.mean_x = means(0);
                cell.mean_y = means(1);
                cell.mean_z = means(2);
                cell.occupancy = map_vector[cell_idx]->getOccupancyRescaled();
                Eigen::Matrix3d cov = map_vector[cell_idx]->getCov();
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        cell.cov_matrix.push_back(cov(i, j));
                    }
                }
                cell.N = map_vector[cell_idx]->getN();
            }
            msg.cells.push_back(cell);
        }
        //		delete map_vector[cell_idx];
    }
    return true;
}

/*
Coloring:
0: single
1: standard
2: cluster
3: update
*/
inline bool toRGBMessage(NDTMap* map,
                         ndt_map_lamide::NDTMapRGBMsg& msg,
                         std::string frame_name,
                         int color = 1)
{
    std::vector<perception_oru::NDTCell*> map_vector = map->getAllInitializedCells();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_name; // is it in *map?
    if (!map->getGridSizeInMeters(msg.x_size, msg.y_size, msg.z_size))
    {
        ROS_ERROR("NO GRID SIZE");
        return false;
    }
    if (!map->getCentroid(msg.x_cen, msg.y_cen, msg.z_cen))
    {
        ROS_ERROR("NO GRID CENTER");
        return false;
    }
    if (!map->getCellSizeInMeters(msg.x_cell_size, msg.y_cell_size, msg.z_cell_size))
    {
        ROS_ERROR("NO CELL SIZE");
        return false;
    }
    for (int cell_idx = 0; cell_idx < map_vector.size(); cell_idx++)
    {
        if (map_vector[cell_idx] != NULL)
        { // we send intialized cells

            ndt_map_lamide::NDTCellRGBMsg cell;
            cell.hasGaussian_ = map_vector[cell_idx]->hasGaussian_;

            cell.center_x = map_vector[cell_idx]->getCenter().x;
            cell.center_y = map_vector[cell_idx]->getCenter().y;
            cell.center_z = map_vector[cell_idx]->getCenter().z;

            cell.occupancy = map_vector[cell_idx]->getOccupancyRescaled();
            cell.occupancy_raw = map_vector[cell_idx]->getOccupancy();

            float r, g, b;
            getColor(color, map_vector[cell_idx], r, g, b);

            cell.red = r;
            cell.green = g;
            cell.blue = b;
            cell.label = map_vector[cell_idx]->getLabel();
            cell.label_weight = map_vector[cell_idx]->getLabelWeight();
            cell.label_time = map_vector[cell_idx]->getLabelTime();

            if (map_vector[cell_idx]->hasGaussian_)
            { // we only send a cell with gaussian

                Eigen::Vector3d means = map_vector[cell_idx]->getMean();
                cell.mean_x = means(0);
                cell.mean_y = means(1);
                cell.mean_z = means(2);
                cell.occupancy = map_vector[cell_idx]->getOccupancyRescaled();
                Eigen::Matrix3d cov = map_vector[cell_idx]->getCov();
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        cell.cov_matrix.push_back(cov(i, j));
                    }
                }
                cell.N = map_vector[cell_idx]->getN();
            }
            msg.cells.push_back(cell);
        }
        //		delete map_vector[cell_idx];
    }
    return true;
}
/**
 *
 * \brief from message to NDTMap object
 * \details Converts ndt map message into a NDTMap object
 * @param[in,out] idx Pointer to lazy grid of the new NDTMap
 * @param[out] map Pointer to NDTMap object
 * @param[in] msg message to be converted
 * @param[out] frame_name name of the coordination frame of the map
 * @param[in] dealloc if set to true, the NDTMap with deallocate the memory of LazyGrid in its
 * destructor. Default set to false.
 * @param[in] get_cell_with_no_gaussian if set to true, cell with no gaussian but initialized are
 * added to the map. Usefull to keep NDT-OM
 *
 */
inline bool fromMessage(LazyGrid*& idx,
                        NDTMap*& map,
                        ndt_map_lamide::NDTMapRGBMsg msg,
                        std::string& frame_name,
                        bool dealloc = false,
                        bool get_cell_with_no_gaussian = true)
{
    if (!(msg.x_cell_size == msg.y_cell_size && msg.y_cell_size == msg.z_cell_size))
    { // we assume that voxels are cubes
        ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE");
        return false;
    }
    idx = new LazyGrid(msg.x_cell_size);
    map = new NDTMap(idx, msg.x_cen, msg.y_cen, msg.z_cen, msg.x_size, msg.y_size, msg.z_size,
                     dealloc);
    frame_name = msg.header.frame_id;

    for (int itr = 0; itr < msg.cells.size(); itr++)
    {
        if (msg.cells[itr].hasGaussian_ == true)
        {
            Eigen::Vector3d mean;
            Eigen::Matrix3d cov;
            mean << msg.cells[itr].mean_x, msg.cells[itr].mean_y, msg.cells[itr].mean_z;
            int m_itr = 0;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cov(i, j) = msg.cells[itr].cov_matrix[m_itr];
                    m_itr++;
                }
            }
            map->addDistributionToCell(cov, mean, msg.cells[itr].N);
            //			std::cout << "Adding initialized cell with gaussian at " << mean(0) << " "
            //<< mean(1) << " " << mean(2) << std::endl;
        }
        else
        {
            if (get_cell_with_no_gaussian == true)
            {
                //				std::cout << "Adding initialized cell with no gaussian" <<
                // std::endl;
                perception_oru::NDTCell* ptCell;
                pcl::PointXYZL point;
                // 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
                point.x = msg.cells[itr].center_x;
                // 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
                point.y = msg.cells[itr].center_y;
                // 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
                point.z = msg.cells[itr].center_z;
                // 			std::cout << "Getting the Cell" << std::endl;
                map->getCellAtAllocate(point, ptCell);
                // 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
                //				ptCell->updateOccupancy(-0.2);
                // 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
                //				if(ptCell->getOccupancy()<=0)
                ptCell->hasGaussian_ = false;
                ptCell->setOccupancy(msg.cells[itr].occupancy_raw);
                //				std::cout << "Occupancy " << ptCell->getOccupancy() << std::endl;

                int occ_rescaled = ptCell->getOccupancyRescaled() * 100;
                int occ_map = msg.cells[itr].occupancy * 100;

                assert(occ_rescaled == occ_map);
                // 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
            }
        }
    }
    return true;
}
/**
 *
 * \brief builds ocuupancy grid message
 * \details Builds 2D occupancy grid map based on 2D NDTMap
 * @param[in] ndt_map 2D ndt map to conversion
 * @param[out] occ_grid 2D cost map
 * @param[in] resolution desired resolution of occupancy map
 * @param[in] name of cooridnation frame for the map (same as the NDT map has)
 *
 */
inline bool toOccupancyGrid(NDTMap* ndt_map,
                            nav_msgs::OccupancyGrid& occ_grid,
                            double resolution,
                            std::string frame_id,
                            double scaling_factor_gaussian = 0.05,
                            bool use_euclidean_distance = false)
{ // works only for 2D case
    double size_x, size_y, size_z;
    int size_x_cell_count, size_y_cell_count;
    double cen_x, cen_y, cen_z;
    double orig_x, orig_y;
    ndt_map->getGridSizeInMeters(size_x, size_y, size_z);
    ndt_map->getCentroid(cen_x, cen_y, cen_z);
    orig_x = cen_x - size_x / 2.0;
    orig_y = cen_y - size_y / 2.0;
    size_x_cell_count = int(size_x / resolution);
    size_y_cell_count = int(size_y / resolution);
    occ_grid.info.width = size_x_cell_count;
    occ_grid.info.height = size_y_cell_count;
    occ_grid.info.resolution = resolution;
    occ_grid.info.map_load_time = ros::Time::now();
    occ_grid.info.origin.position.x = orig_x;
    occ_grid.info.origin.position.y = orig_y;
    occ_grid.header.stamp = ros::Time::now();
    occ_grid.header.frame_id = frame_id;
    // for(double py=orig_y+resolution/2.0;py<orig_y+size_x;py+=resolution){
    //   for(double px=orig_x+resolution/2.0;px<orig_x+size_x;px+=resolution){
    for (int iy = 0; iy < size_y_cell_count; iy++)
    {
        for (int ix = 0; ix < size_x_cell_count; ix++)
        {
            double px = orig_x + resolution * ix + resolution * 0.5;
            double py = orig_y + resolution * iy + resolution * 0.5;

            pcl::PointXYZL pt;
            pt.x = px;
            pt.y = py;
            pt.z = 0;
            perception_oru::NDTCell* cell;
            if (!ndt_map->getCellAtPoint(pt, cell))
            {
                occ_grid.data.push_back(-1);
            }
            else if (cell == NULL)
            {
                occ_grid.data.push_back(-1);
            }
            else
            {
                /* ORU_TODO Will be problematic around wall:
                 * It might put the cell to -1 (as in occupied are unknown since it's not close
                 * enough to the wall to be occupied However, if the cells around are seen empty,
                 * then that cell should be empty also. TO FIX
                 */
                //          if(cell->hasGaussian_) {
                Eigen::Vector3d vec(pt.x, pt.y, pt.z);
                vec = vec - cell->getMean();

                double likelihood = 0;
                if (use_euclidean_distance)
                {
                    likelihood = vec.norm();
                }
                else
                {
                    likelihood = vec.dot(cell->getInverseCov() * vec);
                }
                //	          double likelihood = vec.dot(cell->getInverseCov() * vec);
                //	          double likelihood = vec.norm();
                char s_likelihood;
                if (cell->getOccupancy() != 0.0)
                {
                    if (cell->getOccupancy() > 0.0)
                    {
                        if (std::isnan(likelihood))
                            s_likelihood = -1;
                        //			          std::cout << "SCALIN GAUSSIAN OCC " <<
                        // scaling_factor_gaussian << std::endl;

                        likelihood = exp(-likelihood * scaling_factor_gaussian);
                        //			          likelihood = (0.1 + 0.9 * likelihood);
                        s_likelihood = char(likelihood * 100.0);
                        if (likelihood > 1.0)
                            s_likelihood = 100;
                        occ_grid.data.push_back(s_likelihood);
                    }
                    else
                    {
                        //			          std::cout << "Empty cell in occupancy !" << std::endl;
                        occ_grid.data.push_back(0);
                    }
                }
                else
                {
                    occ_grid.data.push_back(-1);
                }
                //          }
                //          else{
                //	          std::cout << "ADDING CELL WITH NO GAUSSIAN " << std::endl;
                //	          if (cell->getOccupancy() != 0.0) {
                //		          if (cell->getOccupancy() > 0.0) {
                //			          double occ_val = cell->getOccupancyRescaled() * 100;
                //			          std::cout << "With value " << occ_val << std::endl;
                //			          occ_grid.data.push_back(occ_val);
                //		          } else {
                //			          std::cout << "Empty CELL :D ! " << std::endl;
                //			          occ_grid.data.push_back(0);
                //		          }
                //	          } else {
                //		          occ_grid.data.push_back(-1);
                //	          }
                //
                //          }
            }
        }
    }
    return true;
}

/**
 *
 * \brief builds ocuupancy grid message
 * \details Builds 2D occupancy grid map based on 2D NDTMap
 * @param[in] ndt_map 2D ndt map to conversion
 * @param[out] occ_grid 2D cost map
 * @param[in] resolution desired resolution of occupancy map
 * @param[in] name of cooridnation frame for the map (same as the NDT map has)
 *
 */
inline bool toOccupancyGrid(const boost::shared_ptr<perception_oru::NDTMap>& ndt_map,
                            nav_msgs::OccupancyGrid& occ_grid,
                            double resolution,
                            std::string frame_id)
{ // works only for 2D case
    double size_x, size_y, size_z;
    int size_x_cell_count, size_y_cell_count;
    double cen_x, cen_y, cen_z;
    double orig_x, orig_y;
    ndt_map->getGridSizeInMeters(size_x, size_y, size_z);
    ndt_map->getCentroid(cen_x, cen_y, cen_z);
    orig_x = cen_x - size_x / 2.0;
    orig_y = cen_y - size_y / 2.0;
    size_x_cell_count = int(size_x / resolution);
    size_y_cell_count = int(size_y / resolution);
    occ_grid.info.width = size_x_cell_count;
    occ_grid.info.height = size_y_cell_count;
    occ_grid.info.resolution = resolution;
    occ_grid.info.map_load_time = ros::Time::now();
    occ_grid.info.origin.position.x = orig_x;
    occ_grid.info.origin.position.y = orig_y;
    occ_grid.header.stamp = ros::Time::now();
    occ_grid.header.frame_id = frame_id;
    // for(double py=orig_y+resolution/2.0;py<orig_y+size_x;py+=resolution){
    //   for(double px=orig_x+resolution/2.0;px<orig_x+size_x;px+=resolution){
    for (int iy = 0; iy < size_y_cell_count; iy++)
    {
        for (int ix = 0; ix < size_x_cell_count; ix++)
        {
            double px = orig_x + resolution * ix + resolution * 0.5;
            double py = orig_y + resolution * iy + resolution * 0.5;

            pcl::PointXYZL pt;
            pt.x = px;
            pt.y = py;
            pt.z = 0;
            perception_oru::NDTCell* cell;
            if (!ndt_map->getCellAtPoint(pt, cell))
            {
                occ_grid.data.push_back(-1);
            }
            else if (cell == NULL)
            {
                occ_grid.data.push_back(-1);
            }
            else
            {
                Eigen::Vector3d vec(pt.x, pt.y, pt.z);
                vec = vec - cell->getMean();
                double likelihood = vec.dot(cell->getInverseCov() * vec);
                char s_likelihood;
                if (cell->getOccupancy() != 0.0)
                {
                    if (cell->getOccupancy() > 0.0)
                    {
                        if (std::isnan(likelihood))
                            s_likelihood = -1;
                        likelihood = exp(-likelihood / 2.0) + 0.1;
                        likelihood = (0.5 + 0.5 * likelihood);
                        s_likelihood = char(likelihood * 100.0);
                        if (likelihood > 1.0)
                            s_likelihood = 100;
                        occ_grid.data.push_back(s_likelihood);
                    }
                    else
                    {
                        occ_grid.data.push_back(0);
                    }
                }
                else
                {
                    occ_grid.data.push_back(-1);
                }
            }
        }
    }
    return true;
}
} // namespace perception_oru
#endif
