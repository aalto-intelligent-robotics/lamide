
#include "graph_map_lamide/GetGraphMap.h"
#include "graph_map_lamide/GraphMapMsg.h"
#include "graph_map_lamide/factor.h"
#include "graph_map_lamide/graph_map_conversions.h"
#include "graph_map_lamide/graph_map_fuser.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/graphfactory.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/program_options.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace pogm = perception_oru::graph_map;
namespace gm = graph_map;
using graph_map::GraphMapMsg;
GraphMapMsg msg;

bool callback(gm::GetGraphMap::Request& req, gm::GetGraphMap::Response& res)
{
    res.graphmap = msg;
    return true;
}

int main(int argn, char* argv[])
{

    ros::init(argn, argv, "graph_map_server");
    std::string file_name, map_topic;

    pogm::GraphMapNavigatorPtr graph_map_ptr;
    /* if (argn == 2) {
      std::cout << "Usage cliffmap_server <file_name>" << std::endl;
      return -1;
    }*/

    ros::NodeHandle nh("~");

    nh.param<std::string>("map_topic", map_topic, "/maps/map_velodyne");
    nh.param<std::string>("file_name", file_name, "ndt_map.map");
    //FIXME: broken
    pogm::LoadGraphMap(file_name, file_name, graph_map_ptr);
    ros::Publisher graph_map_pub = nh.advertise<GraphMapMsg>(map_topic, 10);
    ROS_INFO("Graph map will be published when there is a subscriber.");
    ros::ServiceServer service_ = nh.advertiseService("get_graphmap", callback);
    ToMessage(msg, graph_map_ptr);
    while (ros::ok())
    {
        if (graph_map_pub.getNumSubscribers() > 0)
            graph_map_pub.publish(msg);
        ros::spinOnce();
    }
}
