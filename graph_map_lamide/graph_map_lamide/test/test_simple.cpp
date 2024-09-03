#include "ndt_map_lamide/ndt_map.h"
#include "pcl/features/feature.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "ros/ros.h"

#include <cstdio>
#include <cstring>
#include <ndt_map_lamide/lazy_grid.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/ndt_conversions.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/pointcloud_utils.h>
#include <string>

int main(int argc, char** argv)
{

    std::cout << "Hello13" << std::endl;
    perception_oru::NDTMap* map = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
    map->initialize(0.0, 0.0, 0.0, 10, 10, 10, 0.6, 0.4, 0.2, 0.7, 0.3, 0.8, 1.0, 0.5);

    std::cout << "Bye3" << std::endl;
}
