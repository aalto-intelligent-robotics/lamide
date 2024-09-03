
/*##############################################################################################################################
### BY DANIEL ADOLFSSON
### What? This program inputs a rosbag and writes <nav_msgs/odometry> into the tf-tree.
###
### Example 1: Given a bag file located at /home/$USER/Documents/001.bag which contain odometrt topic "/odom"
### $ rosrun ndt_offline tf_writer --bag-file-path 2012-01-08_vel_odom.bag --odom-msg-topics /odom
### // tf_writer will create the file /home/$USER/Documents/001.bag_edited.bag containing a tf tree
###
### Example 2: Directory /home/$USER/Documents/bagfiles contain bag files with nav_msgs/odometry on the topic /odom (parent frame="/odom" & child frame="/base_footprint")
### Execute:  rosrun ndt_offline tf_writer --directory /home/$USER/Documents --odom-msg-topics /odom
### Comments: tf_writer will explicitly write the transformation between /odom and /base_footprint into the tf tree for all bag files.
### To force renaming of the frames, use the options: 'odometry-child-frames' and 'odometry-child-frames'
###rosrun ndt_offline tf_writer --bag-file-path `pwd`/001.bag --odom-msg-topics /robot3/control/odom /robot3/control/state --odometry-frames robot3/odom /robot3/odom_gt  --odometry-child-frames robot3/base_link robor3/base_link_gt
*/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/message_filter.h>
#include <boost/program_options.hpp>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>
#include <ndt_offline_lamide/convertvelodynebags.h>
#include <eigen_conversions/eigen_msg.h>
#include <angles/angles.h>
#include <velodyne_msgs/VelodyneScan.h>
#include "tf_conversions/tf_kdl.h"
#include <ros/package.h>
#include "ndt_generic_lamide/sensors_utils.h"
#include "string"
#include <fstream>

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

using namespace std;
namespace po = boost::program_options;


bool create_odom = true, create_tf = true;
bool interactive;
double sensor_time_offset;

// This will underestimate the yaw.
class tfWriter {
public:
    tfWriter(const string &filename, const std::vector <std::string> &topics,
             const std::vector <std::string> &child_frames, const std::vector <std::string> &odometry_frames,
             const std::string &transform_file) {
        topics_ = topics;
        // topics_to_keep_ = topics_to_keep;
        odometry_child_frames_ = child_frames;
        odometry_frames_ = odometry_frames;
        transform_file_=transform_file;

        bag_in_.open(filename, rosbag::bagmode::Read);
        bag_output_.open(filename + "_edited.bag", rosbag::bagmode::Write);

        view_ = new rosbag::View(bag_in_);
        cout << "Messages found :" << view_->size() << endl;
        if (view_->size() == 0)
            exit(0);
        I = view_->begin();
    }

    void ConvertAll() {
        int messages_converted = 0, iteration = 0;
        ros::Time t_last_print = ros::Time::now();
        std::ifstream infile(transform_file_);
        double time, x, y, z, qx, qy, qz, qw;
        std::string parent, child, line;

        while (infile >> time>> x>> y>> z>> qx>> qy>> qz>> qw>> parent>> child ){
            geometry_msgs::TransformStamped msg_odom;
            tf::tfMessage tf_msg;
            ros::Time t(time);

            tf::Transform transform;
            transform.setOrigin( tf::Vector3(x, y, z) );
            tf::Quaternion q(qx, qy, qz, qw);
            transform.setRotation(q);
            tf::transformStampedTFToMsg(tf::StampedTransform(transform, t, parent, child), msg_odom);
            tf_msg.transforms.push_back(msg_odom);
            bag_output_.write(std::string("/tf"), t, tf_msg);
            messages_converted++;
            //bag_output_.write(m.getTopic(), m.getTime(), m);
        }

        cout << endl << messages_converted << " msg's converted" << endl;
        cout << "Closing file...\n" << endl;
        bag_in_.close();
        bag_output_.close();
    }

    void printProgress(double percentage) {
        int val = (int) (percentage * 100);
        int lpad = (int) (percentage * PBWIDTH);
        int rpad = PBWIDTH - lpad;
        printf("\r%3d%% [%.*s>%*s]", val, lpad, PBSTR, rpad, "");
        fflush(stdout);
    }

private:
    std::vector <std::string> topics_;
    std::vector <std::string> topics_to_keep_;
    std::vector <std::string> odometry_child_frames_;
    std::vector <std::string> odometry_frames_;
    rosbag::Bag bag_in_, bag_output_;
    rosbag::View *view_;
    rosbag::View::iterator I;
    std::string transform_file_;
    //geometry_msgs::TransformStamped msg_sensor_;
};

bool LocateRosBagFilePaths(const std::string &folder_name, std::vector <std::string> &scanfiles) {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(folder_name.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_name[0] == '.') continue;
            char tmpcname[400];
            snprintf(tmpcname, 399, "%s/%s", folder_name.c_str(), ent->d_name);
            std::string tmpfname = tmpcname;
            scanfiles.push_back(tmpfname);
        }
        closedir(dir);
    } else {
        std::cerr << "Could not parse dir name\n";
        return false;
    }
}

void eraseAllSubStr(std::string & mainStr, const std::string & toErase)
{
	size_t pos = std::string::npos;

	// Search for the substring in string in a loop untill nothing is found
	while ((pos  = mainStr.find(toErase) )!= std::string::npos)
	{
		// If found then erase it from string
		mainStr.erase(pos, toErase.length());
	}
}

int main(int argc, char **argv) {

    ros::Time::init();
    Eigen::Vector3d transl;
    Eigen::Vector3d euler;
    tf::Transform tf_sensor_pose;
    string inbag_name, directory, trasform_file, trasform_directory;
    std::vector <std::string> odom_msg_topics;
    std::vector <std::string> topics;
    std::vector <std::string> odometry_child_frame_id, odometry_frame_id;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("debug", "additional output")
            ("bag-file-path", po::value<string>(&inbag_name)->default_value(""), "bag file to extend with tf")
            ("directory", po::value<string>(&directory)->default_value(""), "use directory istead of single file")
            ("odom-msg-topics", po::value < std::vector < std::string > > (&odom_msg_topics)->multitoken(),
             "topic of odometry message which will be copied to tf tree")
            ("odometry-child-frames",
             po::value < std::vector < std::string > > (&odometry_child_frame_id)->multitoken(),
             "Force odometry child frames (often refered to ase base_link). Keep field empty if odometry frame should be copied from odometry message child.")
            ("transform-file", po::value < std::string >  (&trasform_file))
            ("transform-directory", po::value < std::string > (&trasform_directory))
            ("odometry-frames", po::value < std::vector < std::string > > (&odometry_frame_id)->multitoken(),
             "Force odometry frames. Keep field empty if odometry frame should be copied from odometry message.");


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || (inbag_name.empty() && directory.empty())) {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("odom-msg-topics"))
        odom_msg_topics = vm["odom-msg-topics"].as < vector < std::string > > ();

    if (vm.count("odometry-child-frames"))
        odometry_child_frame_id = vm["odometry-child-frames"].as < vector < std::string > > ();

    if (vm.count("odometry-frames"))
        odometry_frame_id = vm["odometry-frames"].as < vector < std::string > > ();

//    if (odom_msg_topics.size() == 0) {
//        std::cerr
//                << "Please provide the arguments \"--odom-msg-topics\" and \"--tf-odom-links\" as vectors of equal size, abort program"
//                << std::endl;
//        exit(0);
//    }
//    if (odometry_child_frame_id.size() != odom_msg_topics.size()) {
//        odometry_child_frame_id.resize(odom_msg_topics.size(), std::string(""));
//        cerr
//                << "Size of \"odometry-child-frames\" not matching \"odom-msg-topics\", will copy odometry message frame id"
//                << endl;
//    }
//    if (odometry_frame_id.size() != odom_msg_topics.size()) {
//        odometry_frame_id.resize(odom_msg_topics.size(), std::string(""));
//        cerr
//                << "Size of \"odometry-child-frames\" not matching \"odom-msg-topics\", will copy odometry message frame id"
//                << endl;
//    }

//    for (int i = 0; i < odom_msg_topics.size(); i++) {
//        std::cout << "Converting \"" << odom_msg_topics[i] << "\" => \"" << odometry_child_frame_id[i] << "\"" << endl;
//    }
//    std::cout << endl;

    if (!directory.empty()) {
        std::vector <std::string> paths;
        bool status = LocateRosBagFilePaths(directory, paths);
        for (auto &&path: paths) {
            cout << "Converting file: \"" << path << "\"" << endl;
            std::string t_path=path;
            eraseAllSubStr(t_path,directory);
            t_path=t_path.substr(0, t_path.find("."));
            t_path=trasform_directory+"/"+t_path+"_corrections.txt";
            cout << "Coordinates from file: \"" << t_path << "\"" << endl;
            tfWriter writer(path, odom_msg_topics, odometry_child_frame_id, odometry_frame_id, t_path);
            writer.ConvertAll();
        }
    }
    if (!inbag_name.empty()) {
        tfWriter writer(inbag_name, odom_msg_topics, odometry_child_frame_id, odometry_frame_id, trasform_file);
        writer.ConvertAll();
    }


    return 0;
}
