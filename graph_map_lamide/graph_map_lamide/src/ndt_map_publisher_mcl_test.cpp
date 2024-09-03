#include "ndt_map_lamide/ndt_conversions.h"

#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mcl_init_node");
    ros::NodeHandle param("~");
    /*while( (int)ros::Time(0).toSec() == 0 and ros::ok()){
        std::cout << "Not good " << ros::Time(0) << std::endl;
    }*/

    std::string file =
        "/home/malcolm/AASS/Datasets/Gustav/OFFLINETEST/bag/basement2d_laser_gustav_map_r05.jff";
    // 	map.loadFromJFF(file.c_str());
    double resolution = 0.2;
    auto mapGrid = new perception_oru::LazyGrid(resolution);
    perception_oru::NDTMap map(mapGrid);
    if (map.loadFromJFF(file.c_str()) < 0)
        std::cout << "File didn't load" << std::endl;
    std::cout << "File loaded" << std::endl;

    ndt_map_lamide::NDTMapRGBMsg mapmsg;
    perception_oru::toRGBMessage(&map, mapmsg, "world");

    ros::Publisher ndt_map_pub = param.advertise<ndt_map_lamide::NDTMapRGBMsg>("ndt_map_init_mcl", 10);

    while (ros::ok())
    {
        ros::spinOnce();
        std::cout << "Enter anything to publish the map" << std::endl;
        int a;
        std::cin >> a;
        ndt_map_pub.publish(mapmsg);
        std::cout << "Map Pubkished" << std::endl;
    }

    return 0;
}
