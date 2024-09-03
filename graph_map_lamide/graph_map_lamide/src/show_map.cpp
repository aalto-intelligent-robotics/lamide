// Workaround for lz4 conflicting declaration -----------------------------//
// see https://github.com/ethz-asl/lidar_align/issues/16#issuecomment-758960226
#define LZ4_stream_t                        LZ4_stream_t_deprecated
#define LZ4_resetStream                     LZ4_resetStream_deprecated
#define LZ4_createStream                    LZ4_createStream_deprecated
#define LZ4_freeStream                      LZ4_freeStream_deprecated
#define LZ4_loadDict                        LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue          LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict                        LZ4_saveDict_deprecated
#define LZ4_streamDecode_t                  LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue               LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode              LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode                LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode                 LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue        LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue        LZ4_decompress_fast_continue_deprecated
#include <rosbag/bag.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue
//-------------------------------------------------------------------------//

#include "graph_map_lamide/graph_map_fuser.h"

#include <ros/ros.h>
// #include <rosbag/bag.h>
#include "eigen_conversions/eigen_msg.h"
#include "message_filters/subscriber.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"

#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/ndt_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/view.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>
// #include <ndt_generic_lamide/gnuplot-iostream.h>
#include "boost/serialization/serialization.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "graph_map_lamide/graph_optimization.h"
#include "graph_map_lamide/lidarUtils/lidar_utilities.h"
#include "graph_map_lamide/visualization/graph_plot.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "ros/publisher.h"

#include <graph_map_lamide/graphfactory.h>
#include <graph_map_lamide/ndt/ndtd2d_reg_type.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace perception_oru::graph_map;

/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */

class OpenGraphMap
{

protected:
    // Our NodeHandle

public:
    // Constructor
    OpenGraphMap(ros::NodeHandle* param_nh,
                 const string& file_path,
                 const string& file_name,
                 const bool compare,
                 const string& comp_file_path = "",
                 const string& comp_file_name = "")
    {
        param_nh_ = param_nh;
        file_path_ = file_path;
        file_name_ = file_name;
        comp_file_name_ = comp_file_name;
        has_comp_ = compare;
        if (has_comp_ and comp_file_path.empty())
        {
            comp_file_path_ = file_path;
        }

        bool check_for_broken_submaps = false;
        param_nh_->param<bool>("check_for_broken_submaps", check_for_broken_submaps, false);
        param_nh_->param<int>("color", color_, 1);

        LoadGraphMap(file_path, file_name, graph_map_, check_for_broken_submaps);
        if (has_comp_)
        {
            LoadGraphMap(comp_file_path, comp_file_name, comparison_map_, check_for_broken_submaps);
            color_ = 100;
        }
        graph_size_ = graph_map_->Size();
        cout << "Loaded: " << graph_map_->ToString() << endl;
        vis = new graphVisualization(graph_map_, true, true, true, color_);
        robot_pose_pub_ = param_nh_->advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
        sub = param_nh_->subscribe("/initialpose", 1, &OpenGraphMap::pose_callback, this);
        marker_ = plotmarker::point;
        vis->SetMarkerType(marker_);
    }

    void processFrame()
    {
        static bool first_run = true;
        bool action = false;
        m.lock();
        if (first_run)
        {
            usleep(3 * 1000 * 1000);
            bool success = graph_map_->SwitchToMapNode(graph_map_->GetNode(target_node));
            visualize();
        }
        else if (got_pose_target)
        {
            graph_map_->SwitchToClosestMapNode(pose_target, 111);
            got_pose_target = false;
            visualize();
            action = true;
        }
        else if (got_keyboard_target)
        {
            bool success = graph_map_->SwitchToMapNode(graph_map_->GetNode(target_node));
            got_keyboard_target = false;
            visualize();
            action = true;
        }

        /*if(Save_submap_metadata_){
          saveSubmapMetadata();
          Save_submap_metadata_=false;
          action=true;
        }*/
        m.unlock();
    }

    void SetFilePath(std::string& path)
    {
        output_path = path;
    }

    void visualize()
    {
        graph_map_->m_graph.lock();
        vis->PlotLinks(target_node);
        vis->PlotTrajectory();
        vis->PlotCurrentMap();
        vis->PlotAllClouds();
        vis->PlotActiveNodesCloud();
        graph_map_->m_graph.unlock();
    }
    void setMarker(PlotMarker marker)
    {
        marker_ = marker;
    }

    bool replace(std::string& str, const std::string& from, const std::string& to)
    {
        size_t start_pos = str.find(from);
        if (start_pos == std::string::npos)
            return false;
        str.replace(start_pos, from.length(), to);
        return true;
    }

    void printInfo()
    {
        std::cout << std::endl;
        std::cout << "Welcome to the ShowMap tool which visualizes .MAP files in RViz.\n Use the "
                     "\"2D Pose Estimate\" button in RViz to visualize the closest submap"
                  << std::endl;
        std::cout << "The following commands are avaliable in the prompt:" << std::endl;
        std::cout << "\"d\": switch to next submap" << std::endl;
        std::cout << "\"a\": switch to previous submap" << std::endl;
        std::cout << "\"s\": print submap stats" << std::endl;
        std::cout << "\"g\": goto submap" << std::endl;
        std::cout << "\"t\": print dynamic statistics" << std::endl;
        std::cout << "\"y\": switch color" << std::endl;
        if (edit_mode_)
        {
            std::cout << "\"r\": remove non-static objects" << std::endl;
            std::cout << "\"c\": cluster" << std::endl;
            std::cout << "\"v\": view clusters" << std::endl;
            std::cout << "\"b\": cluster non-static" << std::endl;
            if (has_comp_)
            {
                std::cout << "\"n\": compare maps" << std::endl;
                std::cout << "\"m\": compare all maps" << std::endl;
            }
        }
        else
        {
            std::cout << "\"x\": switch edit mode on/off" << std::endl;
        }
        std::cout << "\"e\": exit" << std::endl;
    }

    void KeyboardInputThread()
    {
        char input = ' ';
        printInfo();

        do
        {
            input = ' ';
            std::cin.clear();
            std::cin >> input;
            if (input == 'd')
            {
                target_node++;
            }
            else if (input == 'a')
            {
                target_node--;
            }
            else if (input == 'g')
            {
                int frame;
                std::cout << "Goto frame:" << std::endl;
                std::cin.clear();
                std::cin >> frame;
                target_node = frame;
            }
            else if (input == 'x')
            {
                edit_mode_ = !edit_mode_;
            }
            else if (input == 'r')
            {
                if (edit_mode_)
                {
                    graph_map_->RemoveNonStaticVoxels();
                }
                else
                {
                    std::cout << "edit mode must be enabled first" << std::endl;
                }
            }
            else if (input == 'c')
            {
                if (edit_mode_)
                {
                    graph_map_->ClusterMap(false, true, true);
                }
                else
                {
                    std::cout << "edit mode must be enabled first" << std::endl;
                }
            }
            else if (input == 'b')
            {
                if (edit_mode_)
                {
                    graph_map_->ClusterMap(false, true, false);
                }
                else
                {
                    std::cout << "edit mode must be enabled first" << std::endl;
                }
            }
            else if (input == 'v')
            {
                if (edit_mode_)
                {
                    graph_map_->ClusterMap(true);
                }
                else
                {
                    std::cout << "edit mode must be enabled first" << std::endl;
                }
            }
            else if (input == 'n' and has_comp_)
            {
                if (edit_mode_)
                {
                    bool reg = false;
                    std::cout << "register maps? y/n" << std::endl;
                    input = ' ';
                    std::cin.clear();
                    std::cin >> input;
                    if (input == 'y')
                    {
                        reg = true;
                    }

                    // First (primary) map
                    MapTypePtr primary = graph_map_->GetCurrentMapNode();

                    // Secondary map
                    comparison_map_->SwitchToMapNode(comparison_map_->GetNode(target_node));
                    MapTypePtr secondary = comparison_map_->GetCurrentMapNode();

                    // transform
                    Eigen::Affine3d tf = Eigen::Affine3d::Identity();
                    if (reg)
                    {
                        // Create registrator
                        RegParamPtr raw_params =
                            GraphFactory::CreateRegParam(ndt_d2d_reg_type_name);
                        NDTD2DRegParamPtr params =
                            boost::dynamic_pointer_cast<NDTD2DRegParam>(raw_params);
                        params->resolution = 0.6;
                        params->resolution_local_factor = 1.0;
                        params->matcher2D_ITR_MAX = 100;
                        params->matcher2D_step_control = true;
                        params->matcher2D_n_neighbours = 2;
                        params->multires = false;
                        params->SoftConstraints = false;
                        RegTypePtr rawreg = GraphFactory::CreateRegistrationType(params);
                        NDTD2DRegTypePtr d2d = boost::dynamic_pointer_cast<NDTD2DRegType>(rawreg);

                        // Register map B to A
                        double score = 0;
                        bool success = d2d->RegisterMap2Map(primary, secondary, tf, score);

                        std::cout << "transform b*T" << std::endl;
                        std::cout << tf.matrix()(0, 0) << ", ";
                        std::cout << tf.matrix()(0, 1) << ", ";
                        std::cout << tf.matrix()(0, 2) << ", ";
                        std::cout << tf.matrix()(0, 3) << ", ";
                        std::cout << tf.matrix()(1, 0) << ", ";
                        std::cout << tf.matrix()(1, 1) << ", ";
                        std::cout << tf.matrix()(1, 2) << ", ";
                        std::cout << tf.matrix()(1, 3) << ", ";
                        std::cout << tf.matrix()(2, 0) << ", ";
                        std::cout << tf.matrix()(2, 1) << ", ";
                        std::cout << tf.matrix()(2, 2) << ", ";
                        std::cout << tf.matrix()(2, 3) << ", ";
                        std::cout << tf.matrix()(3, 0) << ", ";
                        std::cout << tf.matrix()(3, 1) << ", ";
                        std::cout << tf.matrix()(3, 2) << ", ";
                    }

                    std::cout << tf.matrix()(3, 3) << std::endl;

                    // Compare
                    // a
                    NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(primary);
                    perception_oru::NDTMap* a = current_map->GetNDTMap();

                    // b
                    NDTMapPtr other_map = boost::dynamic_pointer_cast<NDTMapType>(secondary);
                    perception_oru::NDTMap* b = other_map->GetNDTMap();

                    if (a and b)
                    {
                        perception_oru::ComparisonResult res = a->compare(b, tf, true);
                        res.print();
                        std::cout << "comparison done" << std::endl;
                        visualize();
                    }
                }
                else
                {
                    std::cout << "edit mode must be enabled first" << std::endl;
                }
            }
            else if (input == 'm' and has_comp_)
            {
                if (edit_mode_)
                {
                    bool reg = false;
                    std::cout << "register maps? y/n" << std::endl;
                    input = ' ';
                    std::cin.clear();
                    std::cin >> input;
                    if (input == 'y')
                    {
                        reg = true;
                    }

                    std::vector<perception_oru::ComparisonResult> results;
                    for (int mapnum = 0; mapnum < graph_size_; mapnum++)
                    {
                        // First (primary) map
                        graph_map_->SwitchToMapNode(graph_map_->GetNode(mapnum));
                        MapTypePtr primary = graph_map_->GetCurrentMapNode();

                        // Secondary map
                        comparison_map_->SwitchToMapNode(comparison_map_->GetNode(mapnum));
                        MapTypePtr secondary = comparison_map_->GetCurrentMapNode();

                        // transform
                        Eigen::Affine3d tf = Eigen::Affine3d::Identity();
                        if (reg)
                        {
                            // Create registrator
                            RegParamPtr raw_params =
                                GraphFactory::CreateRegParam(ndt_d2d_reg_type_name);
                            NDTD2DRegParamPtr params =
                                boost::dynamic_pointer_cast<NDTD2DRegParam>(raw_params);
                            params->resolution = 0.6;
                            params->resolution_local_factor = 1.0;
                            params->matcher2D_ITR_MAX = 100;
                            params->matcher2D_step_control = true;
                            params->matcher2D_n_neighbours = 2;
                            params->multires = false;
                            params->SoftConstraints = false;
                            RegTypePtr rawreg = GraphFactory::CreateRegistrationType(params);
                            NDTD2DRegTypePtr d2d =
                                boost::dynamic_pointer_cast<NDTD2DRegType>(rawreg);

                            // Register map B to A
                            double score = 0;
                            bool success = d2d->RegisterMap2Map(primary, secondary, tf, score);

                            std::cout << "transform b*T" << std::endl;
                            std::cout << tf.matrix()(0, 0) << ", ";
                            std::cout << tf.matrix()(0, 1) << ", ";
                            std::cout << tf.matrix()(0, 2) << ", ";
                            std::cout << tf.matrix()(0, 3) << ", ";
                            std::cout << tf.matrix()(1, 0) << ", ";
                            std::cout << tf.matrix()(1, 1) << ", ";
                            std::cout << tf.matrix()(1, 2) << ", ";
                            std::cout << tf.matrix()(1, 3) << ", ";
                            std::cout << tf.matrix()(2, 0) << ", ";
                            std::cout << tf.matrix()(2, 1) << ", ";
                            std::cout << tf.matrix()(2, 2) << ", ";
                            std::cout << tf.matrix()(2, 3) << ", ";
                            std::cout << tf.matrix()(3, 0) << ", ";
                            std::cout << tf.matrix()(3, 1) << ", ";
                            std::cout << tf.matrix()(3, 2) << ", ";
                            std::cout << tf.matrix()(3, 3) << std::endl;
                        }

                        // Compare
                        // a
                        NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(primary);
                        perception_oru::NDTMap* a = current_map->GetNDTMap();

                        // b
                        NDTMapPtr other_map = boost::dynamic_pointer_cast<NDTMapType>(secondary);
                        perception_oru::NDTMap* b = other_map->GetNDTMap();

                        if (a and b)
                        {
                            perception_oru::ComparisonResult res = a->compare(b, tf, true);
                            res.print();
                            std::cout << "comparison done for map node " << mapnum << std::endl;
                            results.push_back(res);
                        }
                    }

                    std::cout << "all comparisons done!" << std::endl;
                    std::cout << std::endl;

                    std::cout << "matlab results" << std::endl;
                    std::cout << "--------------------------------------------------------------"
                              << std::endl;
                    std::cout << "other missing count"
                              << ", ";
                    std::cout << "this missing count"
                              << ", ";
                    std::cout << "both missing count"
                              << ", ";
                    std::cout << "other no distribution count"
                              << ", ";
                    std::cout << "this no distribution count"
                              << ", ";
                    std::cout << "both no distribution count"
                              << ", ";
                    std::cout << "match count"
                              << ", ";
                    std::cout << "total distance" << std::endl;

                    perception_oru::ComparisonResult sumResult;
                    for (unsigned i = 0; i < results.size(); i++)
                    {
                        std::cout << results[i].other_missing_count_ << ", ";
                        std::cout << results[i].this_missing_count_ << ", ";
                        std::cout << results[i].both_missing_count_ << ", ";
                        std::cout << results[i].other_no_distribution_count_ << ", ";
                        std::cout << results[i].this_no_distribution_count_ << ", ";
                        std::cout << results[i].both_no_distribution_count_ << ", ";
                        std::cout << results[i].match_count_ << ", ";
                        std::cout << results[i].total_distance_ << std::endl;
                        sumResult.other_missing_count_ += results[i].other_missing_count_;
                        sumResult.this_missing_count_ += results[i].this_missing_count_;
                        sumResult.both_missing_count_ += results[i].both_missing_count_;
                        sumResult.other_no_distribution_count_ +=
                            results[i].other_no_distribution_count_;
                        sumResult.this_no_distribution_count_ +=
                            results[i].this_no_distribution_count_;
                        sumResult.both_no_distribution_count_ +=
                            results[i].both_no_distribution_count_;
                        sumResult.match_count_ += results[i].match_count_;
                        sumResult.total_distance_ += results[i].total_distance_;
                    }
                    std::cout << "--------------------------------------------------------------"
                              << std::endl;
                    std::cout << std::endl;

                    std::cout << "sum results" << std::endl;
                    sumResult.print();
                    visualize();
                }
                else
                {
                    std::cout << "edit mode must be enabled first" << std::endl;
                }
            }
            else if (input == 't')
            {
                graph_map_->GetDynamicStatistics();
            }
            else if (input == 's')
            {
                graph_map_->GetSigleMapDynamicStatistics();
            }
            else if (input == 'y')
            {
                color_ += 1;
                if (color_ >= 7)
                {
                    if (not has_comp_)
                    {
                        color_ = 0;
                    }
                    else
                    {
                        color_ = 100;
                    }
                }
                if (color_ >= 100)
                {
                    color_ = 0;
                }
                if (color_ < 0)
                {
                    color_ = 0;
                }
                vis->setColor(color_);

                std::cout << "color: ";
                switch (color_)
                {
                    case 0:
                        std::cout << "single" << std::endl;
                        break;
                    case 1:
                        std::cout << "semantic" << std::endl;
                        break;
                    case 2:
                        std::cout << "clusters" << std::endl;
                        break;
                    case 3:
                        std::cout << "update" << std::endl;
                        break;
                    case 4:
                        std::cout << "occupancy" << std::endl;
                        break;
                    case 5:
                        std::cout << "membership" << std::endl;
                        break;
                    case 6:
                        std::cout << "dynamics" << std::endl;
                        break;
                }

                visualize();
            }
            else if (input == 'p')
            {
                /*cout<<"Requested saving metadata"<<endl;
              Save_submap_metadata_=true;*/
            }
            else if (input == 'h')
            {
                //  cout<<help_text<<endl;
                graph_map_->m_graph.lock();
                cout << "optimize" << endl;
                GlobalSubmapOptimisation o(graph_map_);
                o.ComputeAllScore();
                graph_map_->m_graph.unlock();
            }
            else if (input == 'u')
            {
                graph_map_->m_graph.lock();
                graph_map_->UpdatePoseFrame();
                graph_map_->m_graph.unlock();
            }
            else if (input == 's')
            {
                std::cout << "saving the map..." << std::endl;
                SaveGraphMap(file_path_, file_name_);
                std::cout << "saved the map" << std::endl;
            }
            if (input != ' ')
            {
                if (target_node == graph_size_)
                    target_node = 0;
                else if (target_node == -1)
                    target_node = graph_size_ - 1;

                printInfo();

                cout << "target_node=" << target_node << "/" << graph_size_ << endl;
                got_keyboard_target = true;
            }
        } while (input != 'e');
        cout << "Exit" << endl;
        exit(0);
    }

    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& target_pose)
    {
        geometry_msgs::PoseStamped msg_pose;
        msg_pose.pose.orientation = target_pose.pose.pose.orientation;
        msg_pose.pose.position = target_pose.pose.pose.position;
        msg_pose.header.stamp = ros::Time::now();
        msg_pose.header.frame_id = "/world";
        robot_pose_pub_.publish(msg_pose);
        m.lock();
        tf::poseMsgToEigen(target_pose.pose.pose, pose_target);
        got_pose_target = true;
        m.unlock();
    }

    void SaveGraphMap(const std::string& path, const std::string& filename)
    {
        std::string id = graph_map_->getMapFileID();

        // if the path is missing slash, add it
        std::string fixedpath = path;
        if (path[path.size()] != '/')
        {
            fixedpath = path + "/";
        }

        // everything is unloaded
        for (MapNodePtr node : graph_map_->GetMapNodes())
        {
            if (!node->isOnDisk())
            {
                node->unloadToDisk(id);
            }
        }

        std::string completePath = fixedpath + filename;

        // the structure is saved as the main map
        graph_map_->m_graph.lock();
        cout << "-----------------------------Saving---------------------------------\n"
             << graph_map_->ToString() << endl;
        cout << "----------------------------------------------------------------------\nTo file "
                "path:"
             << completePath << endl;
        std::ofstream ofs(completePath);
        boost::archive::binary_oarchive ar(ofs);
        ar << graph_map_;
        ofs.close();
        graph_map_->m_graph.unlock();
    }

    void Quickstats(const std::string& quickstats_file, const std::string& map_name)
    {
        std::cout << "Creating quick stats for " << map_name << std::endl;
        DynamicStatistics stat;
        {
            ndt_generic::PrintInhibitor pi;
            stat = graph_map_->GetDynamicStatistics();
        }

        // overview
        std::ofstream ofs;
        ofs.open(quickstats_file, std::ofstream::out | std::ofstream::app);
        std::vector<std::string> parts = ndt_generic::splitLine(map_name, "_");

        std::stringstream ss;
        ss << ndt_generic::getISOdate() << ", ";
        if (parts.size() > 2)
        {
            ss << parts[0] << ", " << parts[1].substr(0, 2) << ", " << parts[2] << ", ";
        }
        else
        {
            ss << map_name << ", ";
        }
        std::string prefix = ss.str();

        ofs << prefix;
        ofs << stat.serialize() << std::endl;
        ofs.close();

        // details
        ofs.open(quickstats_file + "_submaps", std::ofstream::out | std::ofstream::app);

        for(unsigned int i = 0; i < stat.size(); i++)
        {
            ofs << prefix;
            ofs << i << ", ";
            ofs << stat[i].serialize() << std::endl;
        }
        ofs.close();
    }

private:
    GraphMapNavigatorPtr graph_map_;
    GraphMapNavigatorPtr comparison_map_;
    ros::NodeHandle* param_nh_;
    ros::Subscriber gt_sub;
    ros::Subscriber sub;
    // Components for publishing
    ros::Publisher robot_pose_pub_;
    Eigen::Affine3d pose_;
    boost::mutex m, message_m;
    std::string gt_topic, bag_name;
    ros::Publisher map_publisher_;
    bool got_pose_target = false, got_keyboard_target = false;
    int target_node = 0;
    Eigen::Affine3d pose_target;
    PlotMarker marker_;
    graphVisualization* vis;
    unsigned int graph_size_;
    bool edit_mode_ = false;

    std::string output_path = "";
    std::string file_path_ = "";
    std::string file_name_ = "";
    std::string comp_file_path_ = "";
    std::string comp_file_name_ = "";
    bool has_comp_ = false;
    int color_ = 0;
};

int main(int argc, char** argv)
{
    std::string file_name;
    std::string file_path;
    std::string comp_file_name;
    std::string comp_file_path;
    std::string output_filename;
    std::string quickstats_file;
    bool compare;
    bool quickstats;
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")(
        "file-path",
        po::value<std::string>(&file_path)->default_value(std::string("default-file-path")),
        "name of path to load containing graphMap")(
        "file-name",
        po::value<std::string>(&file_name)->default_value(std::string("default-file-name")),
        "name of file to load containing graphMap")(
        "comp-file-path", po::value<std::string>(&comp_file_path)->default_value(std::string("")),
        "name of path to load comparison graphMap")(
        "comp-file-name", po::value<std::string>(&comp_file_name)->default_value(std::string("")),
        "name of file to load comparison graphMap")(
        "compare", po::value<bool>(&compare)->default_value(false),
        "compare y/n")("output-filepath",
                       po::value<std::string>(&output_filename)
                           ->default_value(std::string("/home/submap_metadata")),
                       "name of file to load containing graphMap")(
        "quickstats", po::value<bool>(&quickstats)->default_value(false), "get stats and quit")(
        "quickstatsfile", po::value<std::string>(&quickstats_file)->default_value(""),
        "file for stats");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        exit(0);
    }
    ros::init(argc, argv, "show_map");
    ros::NodeHandle param("~");

    std::cout << "Attempt to open map: " << file_name << endl;
    ros::Rate loop_rate(10);
    OpenGraphMap t(&param, file_path, file_name, compare, comp_file_path, comp_file_name);
    t.SetFilePath(output_filename);

    if (quickstats)
    {
        t.Quickstats(quickstats_file, file_name);
    }
    else
    {
        std::thread input_th(&OpenGraphMap::KeyboardInputThread, &t);

        while (param.ok())
        {
            t.processFrame();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
