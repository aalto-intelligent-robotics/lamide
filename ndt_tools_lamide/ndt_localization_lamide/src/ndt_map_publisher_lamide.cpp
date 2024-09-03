#include "ros/ros.h"
#include <ndt_map_lamide/ndt_conversions.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/lazy_grid.h>
#include <ndt_map_lamide/pointcloud_utils.h>

#include <ndt_map_lamide/NDTMapMsg.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv){
        ros::init(argc, argv, "ndt_map_publisher");
        ros::NodeHandle nh;
        ros::NodeHandle parameters("~");
        std::string mapFile;
        std::string mapTopic;
        std::string mapFrame;
        double resolution;
        double mapRate;

        parameters.param<std::string>("map_file", mapFile, "file.jff");
        parameters.param<std::string>("map_topic", mapTopic, "NDT_map");
        parameters.param<std::string>("map_frame", mapFrame, "map");
        parameters.param<double>("resolution", resolution, 0.4);
        parameters.param<double>("map_rate", mapRate, 1);

        ros::Publisher map_pub = nh.advertise<ndt_map_lamide::NDTMapRGBMsg>(mapTopic, 1);
        ros::Rate loop_rate(mapRate);
        ndt_map_lamide::NDTMapRGBMsg msg;

        perception_oru::NDTMap nd(new perception_oru::LazyGrid(resolution));
        ROS_INFO_STREAM("loading from jff...\n");
        if(nd.loadFromJFF(mapFile.c_str()) < 0)
                ROS_ERROR_STREAM("loading from jff failed\n");

        perception_oru::toRGBMessage(&nd, msg, mapFrame);
        while(ros::ok()){
                map_pub.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
