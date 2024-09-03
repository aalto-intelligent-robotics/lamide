#include "ndt_offline_lamide/gps_test.h"
bool read_bag(const std::string &bag_path, std::vector<ublox_msgs::NavPOSECEF> &buffer){
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/rover/navposecef"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {
        ublox_msgs::NavPOSECEF::ConstPtr s = m.instantiate<ublox_msgs::NavPOSECEF>();
        if (s != NULL)
            buffer.push_back(*s);
    }
    bag.close();
    std::cout<<"found: "<<buffer.size()<<" messages."<<std::endl;
    return buffer.size()>0; //return true if messages were read
}
void pose_position_from_ecef(nav_msgs::Odometry &pose, ublox_msgs::NavPOSECEF &ecef,std::vector<double> initial){
    pose.pose.pose.position.x=(ecef.ecefX-initial[0])/100;
    pose.pose.pose.position.y=(ecef.ecefY-initial[1])/100;
    pose.pose.pose.position.z=(ecef.ecefZ-initial[2])/100;
    std::cout<<pose.pose.pose.position<<std::endl;
    pose.pose.pose.orientation.w=1;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "gps");
    ros::NodeHandle n;
    ros::Time::init();
    ros::Time tplot=ros::Time::now();
    nav_msgs::Odometry pose;
    std::vector<ublox_msgs::NavPOSECEF> buffer;
    read_bag("/home/niclas/bags/19april/2018-04-19-09-48-45.bag",buffer);
    std::vector<double> initial;
    initial.push_back(buffer[0].ecefX);
    initial.push_back(buffer[0].ecefY);
    initial.push_back(buffer[0].ecefZ);
    ros::Publisher *pub;
    pub=new ros::Publisher();
    *pub= n.advertise<nav_msgs::Odometry> ("/gps_pos", 1000);
    for (int i=0;i<buffer.size();i++){
        tplot=ros::Time::now();
        pose_position_from_ecef(pose,buffer[i],initial);
        pose.header.frame_id="/world";
        pose.header.stamp=tplot;
        pub->publish(pose);
        sleep(1);
    }
}
