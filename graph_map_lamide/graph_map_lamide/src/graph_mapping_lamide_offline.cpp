#include <ndt_fuser_lamide/ndt_fuser_hmt.h>
#include <ndt_generic_lamide/eigen_utils.h>
#include <ndt_generic_lamide/pcl_utils.h>
#include <ndt_offline_lamide/VelodyneBagReader.h>
// PCL specific includes
#include "graph_map_lamide/graph_map_fuser.h"
#include "graph_map_lamide/ndt/ndt_map_param.h"
#include "graph_map_lamide/ndt/ndt_map_type.h"
#include "graph_map_lamide/ndt/ndtd2d_reg_type.h"
#include "graph_map_lamide/ndt_dl/ndtdl_reg_type.h"
#include "graph_map_lamide/ndt_dl/point_curv3.h"
#include "nav_msgs/Odometry.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/io.h"
#include "ndt_offline_lamide/pointcloudbagreader.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <boost/program_options.hpp>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/pointcloud_utils.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf_conversions/tf_eigen.h>
//#include "ndt_offline_lamide/readbagfilegeneric.h"
#include "graph_map_lamide/visualization/graph_visualization.h"
#include "graph_map_custom_msgs_lamide/NNDataSaver.h"
#include "ndt_generic_lamide/io.h"
#include "ndt_generic_lamide/motion_model_3d.h"
#include "ndt_generic_lamide/motionmodels.h"
#include "ndt_generic_lamide/point_types.h"
#include "ndt_generic_lamide/sensors_utils.h"
#include "ndt_offline_lamide/readpointcloud.h"
#include "tf_conversions/tf_eigen.h"
#include "velodyne_pointcloud_oru/rawdata.h"

#include <unistd.h>
namespace po = perception_oru;
namespace pogm = perception_oru::graph_map;
namespace bopo = boost::program_options;
using namespace std;

Eigen::Vector3d transl;
Eigen::Vector3d euler;
std::string map_dir_name = "";
std::string output_dir_name = "";
std::string base_name = "";
std::string dataset = "";
std::string bag_reader_type = "";
// map parameters
int itrs = 0;
int attempts = 1;
int nb_neighbours = 0;
int nb_scan_msgs = 0;
bool use_odometry_source = false;
double lambda = 1.0;
bool visualize = true;
bool visualize_map = true;
bool use_multires = false;
bool beHMT = false;
bool filter_cloud = false;
bool step_control = false;
bool check_consistency = true;
bool registration2d = true;
bool use_submap = true;
double min_keyframe_dist = 0.5;
double min_keyframe_dist_rot_deg = 15;
bool use_keyframe = true;
bool alive = false;
bool save_map = true;
bool gt_mapping = false;
bool disable_reg = false, do_soft_constraints = false;
bool pcl_reader = true;
bool init_map_gt_frame = false;
ros::ServiceClient* client;
perception_oru::MotionModel2d::Params motion_params;
std::string base_link_id = "", gt_base_link_id = "", tf_world_frame = "", odom_parent_link = "",
            odom_gt_parent_link = "";
std::string velodyne_config_file = "";
std::string lidar_topic = "";
std::string velodyne_frame_id = "";
std::string map_type_name = "", registration_type_name = "";
std::string tf_topic = "";
tf::Transform Tsensor_offset_tf;
std::string map_switching_method = "";
ndt_generic::Vector6d init;
ros::NodeHandle* n_ = NULL;
pogm::RegParamPtr regParPtr = NULL;
pogm::MapParamPtr mapParPtr = NULL;
pogm::GraphMapNavigatorParamPtr graphParPtr = NULL;
po::MotionModel3d mot_model;
pogm::graphVisualization* vis;
double sensor_time_offset = 0;
double map_size_xy = 0;
double map_size_z = 0;
double resolution_local_factor = 0;
double max_range = 0, min_range = 0;
double maxRotationNorm = 0;
double compound_radius = -1;
double interchange_radius = 0;
double distance_alpha = 1.0;
double maxTranslationNorm = 0;
double rotationRegistrationDelta = 0;
double sensorRange = 30;
double translationRegistrationDelta = 0;
double resolution = 0;
double hori_min = 0, hori_max = 0;
double z_min = 0, z_max = 0;
unsigned int skip_frame = 20;
double Tmap, Tother;
double th_segment;
bool save_birds_eye = false;
ros::Publisher *gt_pub, *fuser_pub, *cloud_pub, *odom_pub, *cloud_segmented_pub;
nav_msgs::Odometry gt_pose_msg, fuser_pose_msg, odom_pose_msg;
bool use_pointtype_xyzir;
int min_nb_points_for_gaussian;
bool keep_min_nb_points;
bool min_nb_points_set_uniform;
int nb_measurements = 1;
int max_nb_iters = 30;
bool generate_eval_files = false;
bool use_only_static_scans = false;
bool save_used_merged_clouds = false;
bool save_raw_clouds = false;
bool save_graph_cloud = false;
bool maptype_cloud = false;
bool use_gt_data = false;
int nb_frames = 0;
bool disable_submap_orientation = false;
int counter = 0;
bool filter_ground = false;
int cloud_queue_size = 10;
bool unpack_raw = false;
ndt_generic::Vector6d sigma;
ndt_offline::OdometryType odom_type;
Eigen::Affine3d Todom_base_prev, Tgt_base_prev, Tgt_base, Todom_base, Tgt_t0, Todom_t0, fuser_pose,
    fuser_pose_init, Tsensor_offset;
Eigen::MatrixXd predCov;
pogm::GraphMapFuser* fuser_ = NULL;
Eigen::Affine3d offset;
ndt_generic::PointCloudQueue<pcl::PointXYZL>* cloud_queue_birds_view;
ndt_generic::PointCloudQueue<velodyne_pointcloud_oru::PointXYZIR>* cloud_ir_queue;

template <class T> std::string toString(const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error("::toString()");

    return o.str();
}

std::string transformToEvalString(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T)
{
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::digits10);
    Eigen::Quaternion<double> tmp(T.rotation());
    stream << T.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z()
           << " " << tmp.w() << std::endl;
    return stream.str();
}

bool ReadAllParameters(bopo::options_description& desc, int& argc, char*** argv)
{

    double Tx, Ty, Tz;
    double nickes_arg;
    // First of all, make sure to advertise all program options
    desc.add_options()("help", "produce help message")(
        "map-type-name", bopo::value<string>(&map_type_name)->default_value(std::string("ndt_map")),
        "type of map to use e.g. ndt_map or ndt_dl_map (default it default)")(
        "registration-type-name",
        bopo::value<string>(&registration_type_name)->default_value(std::string("ndt_d2d_reg")),
        "type of map to use e.g. ndt_d2d_reg or ndt_dl_reg (default it default)")(
        "visualize", "visualize the output")("visualize-map", "visualize the map")(
        "T-map", bopo::value<double>(&Tmap)->default_value(4.),
        "Time period between plotting of map")("T-links",
                                               bopo::value<double>(&Tother)->default_value(1.),
                                               "Time period betweem plotting of link")(
        "disable-odometry", "dont use odometry as initial guess")("disable-mapping",
                                                                  "build maps from cloud data")(
        "attempts", bopo::value<int>(&attempts)->default_value(1),
        "Total retries of localisation, can be used to generate multiple files")(
        "step-control", "use step control in the optimization (default=false)")(
        "base-name", bopo::value<string>(&base_name)->default_value("off"),
        "prefix for all generated files")(
        "reader-type", bopo::value<string>(&bag_reader_type)->default_value("velodyne_reader"),
        "e.g. velodyne_reader or pcl_reader")(
        "data-set", bopo::value<string>(&dataset)->default_value(std::string("")),
        "choose which dataset that is currently used, this option will assist with assigning the "
        "sensor pose")("dir-name", bopo::value<string>(&map_dir_name),
                       "where to look for ros bags")(
        "map-switching-method",
        bopo::value<string>(&map_switching_method)->default_value("node_position"),
        "where to look for ros bags")(
        "output-dir-name",
        bopo::value<string>(&output_dir_name)->default_value("/home/daniel/.ros/maps"),
        "where to save the pieces of the map (default it ./map)")(
        "map-size-xy", bopo::value<double>(&map_size_xy)->default_value(83.),
        "size of submaps")("map-size-z", bopo::value<double>(&map_size_z)->default_value(6.0),
                           "size of submaps")("itrs", bopo::value<int>(&itrs)->default_value(30),
                                              "number of iteration in the registration")(
        "fuse-incomplete", "fuse in registration estimate even if iterations ran out. may be "
                           "useful in combination with low itr numbers")(
        "filter-cloud",
        "Primary filter FoV  and secondary filter range. Range is already filtered in the reader, "
        "however if unpack-raw is set, this can range filter") // replaces filter-fov (its made more
                                                               // general
        ("hori-max", bopo::value<double>(&hori_max)->default_value(2 * M_PI),
         "the maximum field of view angle horizontal")(
            "hori-min", bopo::value<double>(&hori_min)->default_value(-2 * M_PI),
            "the minimum field of view angle horizontal")(
            "z-min", bopo::value<double>(&z_min)->default_value(-DBL_MAX),
            "minimum height in lidar frame")("z-max",
                                             bopo::value<double>(&z_max)->default_value(DBL_MAX),
                                             "maximum_height in lidar frame")(
            "Dd", bopo::value<double>(&motion_params.Dd)->default_value(1.),
            "forward uncertainty on distance traveled")(
            "Dt", bopo::value<double>(&motion_params.Dt)->default_value(1.),
            "forward uncertainty on rotation")(
            "Cd", bopo::value<double>(&motion_params.Cd)->default_value(1.),
            "side uncertainty on distance traveled")(
            "Ct", bopo::value<double>(&motion_params.Ct)->default_value(1.),
            "side uncertainty on rotation")(
            "Td", bopo::value<double>(&motion_params.Td)->default_value(1.),
            "rotation uncertainty on distance traveled")(
            "Tt", bopo::value<double>(&motion_params.Tt)->default_value(1.),
            "rotation uncertainty on rotation")(
            "tf-base-link",
            bopo::value<std::string>(&base_link_id)->default_value(std::string("/state_base_link")),
            "tf_base_link")(
            "tf-gt-link",
            bopo::value<std::string>(&gt_base_link_id)->default_value(std::string("/base_link")),
            "tf ground truth link")("velodyne-config-file",
                                    bopo::value<std::string>(&velodyne_config_file)
                                        ->default_value(std::string("../config/velo32.yaml")),
                                    "configuration file for the scanner")(
            "tf-world-frame",
            bopo::value<std::string>(&tf_world_frame)->default_value(std::string("/world")),
            "tf world frame")(
            "lidar-topic",
            bopo::value<std::string>(&lidar_topic)->default_value(std::string("/velodyne_packets")),
            "velodyne packets topic used")(
            "velodyne-frame-id",
            bopo::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")),
            "frame_id of the laser sensor")(
            "odom-parent-link",
            bopo::value<std::string>(&odom_parent_link)->default_value(std::string("/world")),
            "frame_id of the laser sensor")(
            "odom-gt-parent-link",
            bopo::value<std::string>(&odom_gt_parent_link)->default_value(std::string("/world")),
            "frame_id of the laser sensor")(
            "alive", "keep the mapper/visualization running even though it is completed (e.g. to "
                     "take screen shots etc.")("nb_neighbours",
                                               bopo::value<int>(&nb_neighbours)->default_value(2),
                                               "number of neighbours used in the registration")(
            "min-range", bopo::value<double>(&min_range)->default_value(0.6),
            "minimum range used from scanner")("max-range",
                                               bopo::value<double>(&max_range)->default_value(130),
                                               "minimum range used from scanner")(
            "save-map", "saves the graph map at the end of execution")(
            "nb_scan_msgs", bopo::value<int>(&nb_scan_msgs)->default_value(1),
            "number of scan messages that should be loaded at once from the bag")(
            "disable-keyframe-update",
            "use every scan to update map rather than update map upon distance traveled")(
            "keyframe-min-distance", bopo::value<double>(&min_keyframe_dist)->default_value(0.5),
            "minimum range used from scanner")(
            "th-segment", bopo::value<double>(&th_segment)->default_value(2.5),
            "th segment")("keyframe-min-rot-deg",
                          bopo::value<double>(&min_keyframe_dist_rot_deg)->default_value(15),
                          "minimum range used from scanner")(
            "gt-mapping", "disable registration and use ground truth as input to mapping")(
            "tf-topic", bopo::value<std::string>(&tf_topic)->default_value(std::string("/tf")),
            "tf topic to listen to")("Tx", bopo::value<double>(&transl[0])->default_value(0.),
                                     "sensor pose - translation vector x")(
            "Ty", bopo::value<double>(&transl[1])->default_value(0.),
            "sensor pose - translation vector y")(
            "Tz", bopo::value<double>(&transl[2])->default_value(0.),
            "sensor pose - translation vector z")("Rex",
                                                  bopo::value<double>(&euler[0])->default_value(0.),
                                                  "sensor pose - euler angle vector x")(
            "Rey", bopo::value<double>(&euler[1])->default_value(0.),
            "sensor pose - euler angle vector y")("Rez",
                                                  bopo::value<double>(&euler[2])->default_value(0.),
                                                  "sensor pose - euler angle vector z")(
            "lambda-sc", bopo::value<double>(&lambda)->default_value(100), "lambda for sc")(
            "sx", bopo::value<double>(&sigma[0])->default_value(0.1), "Lambda sigma - x")(
            "sy", bopo::value<double>(&sigma[1])->default_value(0.1), "Lambda sigma - y")(
            "sz", bopo::value<double>(&sigma[2])->default_value(0.1), "Lambda sigma - z")(
            "sex", bopo::value<double>(&sigma[3])->default_value(0.1), "Lambda sigma - ex")(
            "sey", bopo::value<double>(&sigma[4])->default_value(0.1), "Lambda sigma - ey")(
            "sez", bopo::value<double>(&sigma[5])->default_value(0.1), "Lambda sigma - ez")(
            "init-x", bopo::value<double>(&init[0])->default_value(0.00),
            "init-x")("init-y", bopo::value<double>(&init[1])->default_value(0.0), "init-y")(
            "init-z", bopo::value<double>(&init[2])->default_value(0.0),
            "init-z")("init-ex", bopo::value<double>(&init[3])->default_value(0.0), "init-ex")(
            "init-ey", bopo::value<double>(&init[4])->default_value(0.0),
            "init-ey")("init-ez", bopo::value<double>(&init[5])->default_value(0.0), "init-ez")(
            "skip-frame", bopo::value<unsigned int>(&skip_frame)->default_value(20),
            "sframes to skip before plot map etc.")(
            "sensor-time-offset", bopo::value<double>(&sensor_time_offset)->default_value(0.),
            "timeoffset of the scanner data")(
            "registration3d",
            "registration3d") // Do not limit registration to the plane (x,y,theta)
        ("disable-registration",
         "Disable Registration")("soft-constraints", "Use soft constraints in the registration")(
            "init-pose-gt-frame", "Align first map node frame with ground truth frame")(
            "unpack-non-returns", "Unpack no beam returns")(
            "unpack-raw",
            "No fitering is performed in the reader. This is done if the complete point cloud is "
            "needed")("check-consistency", "if consistency should be checked after registration")(
            "multi-res", "multi resolution registration")(
            "consistency-max-rot", bopo::value<double>(&maxRotationNorm)->default_value(0.8),
            "maxRotationNorm")(
            "consistency-max-dist", bopo::value<double>(&maxTranslationNorm)->default_value(0.4),
            "maxTranslationNorm")("alpha", bopo::value<double>(&distance_alpha)->default_value(0.0),
                                  "alpha in the distance formula") // d=alpha*t+(1-alpha)*k*rot;
        ("translationRegistrationDelta",
         bopo::value<double>(&translationRegistrationDelta)->default_value(1.5),
         "sensorRange")("resolution", bopo::value<double>(&resolution)->default_value(0.4),
                        "resolution of the map")("resolutions",
                                                 bopo::value<std::vector<double>>()->multitoken(),
                                                 "multiple resolutions if dl maps are used")(
            "resolution-local-factors", bopo::value<std::vector<double>>()->multitoken(),
            "multiple resolutions if dl maps are used")(
            "resolution-local-factor",
            bopo::value<double>(&resolution_local_factor)->default_value(1.),
            "resolution factor of the local map used in the match and fusing step")(
            "disable-submaps", "Adopt the sub-mapping technique which represent the global map as "
                               "a set of local submaps")(
            "compound-radius", bopo::value<double>(&compound_radius)->default_value(10.0),
            "Requires sub-mapping enabled, When creating new sub-lamps, information from previous "
            "map is transfered to the new map. The following radius is used to select the map "
            "objects to transfer")(
            "interchange-radius", bopo::value<double>(&interchange_radius)->default_value(10.0),
            "This radius is used to trigger creation or selection of which submap to use")(
            "use-pointtype-xyzir", "If the points to be processed should contain ring and "
                                   "intensity information (velodyne_pointcloud_oru::PointXYZIR)")(
            "min-nb-points-for-gaussian",
            bopo::value<int>(&min_nb_points_for_gaussian)->default_value(3),
            "minimum number of points per cell to compute a gaussian")(
            "keep-min-nb-points", "If the number of points stored in a NDTCell should be cleared "
                                  "if the number is less than min_nb_points_for_gaussian")(
            "min-nb-points-set-uniform",
            "If the number of points of one cell is less than min_nb_points_for_gaussian, set the "
            "distribution to a uniform one (cov = Identity)")(
            "nb-measurements", bopo::value<int>(&nb_measurements)->default_value(1),
            "number of scans collecte at each read iteration")(
            "max-nb-iters", bopo::value<int>(&max_nb_iters)->default_value(30),
            "max number of iterations used in the registration")(
            "generate-eval-files",
            "generate evaluation files at each used scan pose, storing the affine poses")(
            "save-used-merged-clouds",
            "incase of using static scans, multiple scans is used for each update, this will save "
            "the merged clouds")("save-raw-clouds", "save the (unwarped) clouds in sensor frame")(
            "store-points", "store pointclouds used to create the graph_map")(
            "save-graph-cloud",
            "Save a pointcloud corresponding to all the points that was used to update the graph")(
            "disable-unwarp", "dont unwarp distorted poincloud")(
            "save-birds-eye", "Save birds-eye")("disable-submap-orientation",
                                                "Disable orientation to initialize submaps, this "
                                                "might have an impact on mapping quality")(
            "maptype-cloud",
            "With the option save-graph-cloud enabled, this option will use the underlying map "
            "type (such as NDT-OM) to create the pointcloud, rather than using the original "
            "points")("filter-ground", "visualize point cloud which has been filtered");

    // Boolean parameres are read through notifiers
    bopo::variables_map vm;
    bopo::store(bopo::parse_command_line(argc, *argv, desc), vm);
    bopo::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return false;
    }

    keep_min_nb_points = !vm.count("clear-min-nb-points");
    min_nb_points_set_uniform = vm.count("min-nb-points-set-uniform");
    po::NDTCell::setParameters(0.1, 8 * M_PI / 18., 1000, min_nb_points_for_gaussian,
                               keep_min_nb_points, min_nb_points_set_uniform);
    gt_pose_msg.header.frame_id = "/world";
    fuser_pose_msg.header.frame_id = "/world";
    odom_pose_msg.header.frame_id = "/world";

    if (ndt_generic::GetSensorPose(dataset, transl, euler, Tsensor_offset_tf, Tsensor_offset))
    {
        std::cout << "sensor pose from dataset utilized [" << dataset << "]" << endl;
    }

    mapParPtr = pogm::GraphFactory::CreateMapParam(map_type_name); // map_type_name
    regParPtr = pogm::GraphFactory::CreateRegParam(registration_type_name);

    graphParPtr = pogm::GraphMapNavigatorParamPtr(new pogm::GraphMapNavigatorParam());
    if (mapParPtr == NULL || regParPtr == NULL || graphParPtr == NULL)
    {
        std::cout << "Null pointers" << endl;
        exit(0);
    }

    cloud_queue_birds_view = new ndt_generic::PointCloudQueue<pcl::PointXYZL>(
        10); // WARN THIS NEEDS TO BE SET TO 10 FOR KEVINS SOFTWARE
    use_pointtype_xyzir = vm.count("use-pointtype-xyzir");

    cout << "tf-odom: " << odom_parent_link << ", tf-base_link: " << base_link_id
         << ", disable unwarp?" << std::boolalpha << vm.count("disable-unwarp") << endl;

    use_odometry_source =
        !(vm.count("disable-unwarp") || odom_parent_link.empty() || base_link_id.empty());
    use_keyframe = !vm.count("disable-keyframe-update");
    if (!use_odometry_source || vm.count("disable-unwarp"))
    {
        odom_type = ndt_offline::NO_ODOM;
        cout << "No Odometry" << endl;
        use_keyframe = false;
    }
    else
    {
        odom_type = ndt_offline::WHEEL_ODOM;
    }

    velodyne_rawdata_oru::RawData::unpack_invalid_returns = vm.count("unpack-non-returns");
    visualize = vm.count("visualize");
    visualize_map = vm.count("visualize-map");
    unpack_raw = vm.count("unpack-raw");
    filter_cloud = vm.count("filter-cloud");
    step_control = (vm.count("step-control"));
    gt_mapping = vm.count("gt-mapping");
    save_birds_eye = vm.count("save-birdeye");

    check_consistency = vm.count("check-consistency");
    alive = vm.count("alive");
    save_map = vm.count("save-map");
    registration2d = !vm.count("registration3d");
    use_gt_data = gt_base_link_id != "" && odom_parent_link != "";
    init_map_gt_frame = vm.count("init-pose-gt-frame");
    filter_ground = vm.count("filter-ground");
    disable_submap_orientation = vm.count("disable-submap-orientation");

    vector<double> resolutions;
    if (vm.count("resolutions"))
    {
        resolutions = vm["resolutions"].as<vector<double>>();
    }
    if (resolutions.size() != 3)
    {
        resolutions.clear();
        for (int i = 0; i < 3; i++)
        {
            resolutions.push_back(resolution);
        }
    }
    vector<double> resolution_local_factors;
    if (vm.count("resolution-local-factors"))
    {
        resolutions = vm["resolution-local-factors"].as<vector<double>>();
    }
    if (resolution_local_factors.size() != 3)
    {
        resolution_local_factors.clear();
        for (int i = 0; i < 3; i++)
        {
            resolution_local_factors.push_back(resolution_local_factor);
        }
    }

    regParPtr->enable_registration = !vm.count("disable-registration");
    regParPtr->registration2d = registration2d;
    regParPtr->max_rotation_norm = maxRotationNorm;
    regParPtr->max_translation_norm = maxTranslationNorm;
    regParPtr->rotation_registration_delta = rotationRegistrationDelta;
    regParPtr->translation_registration_delta = translationRegistrationDelta;
    regParPtr->sensor_range = max_range;
    regParPtr->map_size_z = map_size_z;
    regParPtr->check_consistency = check_consistency;
    regParPtr->sensor_pose = Tsensor_offset;
    regParPtr->use_initial_guess = !vm.count("disable-odometry");
    mapParPtr->sizez_ = map_size_z;
    mapParPtr->max_range_ = max_range;
    mapParPtr->min_range_ = min_range;
    mapParPtr->store_points = vm.count("store-points");

    graphParPtr->compound_radius = compound_radius;
    graphParPtr->interchange_radius = interchange_radius;
    graphParPtr->alpha = distance_alpha;
    graphParPtr->map_switch_method =
        pogm::GraphMapNavigatorParam::String2SwitchMethod(map_switching_method);
    std::cout << "use keyframe=" << use_keyframe;
    graphParPtr->use_keyframe = use_keyframe;
    graphParPtr->min_keyframe_dist = min_keyframe_dist;
    graphParPtr->min_keyframe_rot_deg = min_keyframe_dist_rot_deg;
    std::cout << "keyframe dist" << graphParPtr->min_keyframe_dist << endl;
    use_submap = !vm.count("disable-submaps");
    graphParPtr->use_submap = use_submap;
    graphParPtr->Tsensor = Tsensor_offset;
    graphParPtr->identity_orientation = disable_submap_orientation;

    mapParPtr->enable_mapping_ = !vm.count("disable-mapping");
    mapParPtr->sizey_ = map_size_xy;
    mapParPtr->sizex_ = map_size_xy;

    GetMotionModel(dataset, mot_model);

    if (pogm::NDTD2DRegParamPtr ndt_reg_ptr =
            boost::dynamic_pointer_cast<pogm::NDTD2DRegParam>(regParPtr))
    {
        ndt_reg_ptr->resolution = resolution;
        ndt_reg_ptr->resolution_local_factor = resolution_local_factor;
        ndt_reg_ptr->matcher2D_ITR_MAX = max_nb_iters;
        ndt_reg_ptr->multires = vm.count("multi-res");
        ndt_reg_ptr->SoftConstraints = vm.count("soft-constraints");
    }
    else if (pogm::NDTDLRegParamPtr ndt_reg_ptr =
                 boost::dynamic_pointer_cast<pogm::NDTDLRegParam>(regParPtr))
    {
        ndt_reg_ptr->resolutions = resolutions;
        ndt_reg_ptr->resolution_local_factors = resolution_local_factors;
        ndt_reg_ptr->matcher2D_ITR_MAX = max_nb_iters;
        ndt_reg_ptr->multires = vm.count("multi-res");
        ndt_reg_ptr->SoftConstraints = vm.count("soft-constraints");
    }

    if (pogm::NDTMapParamPtr ndt_map_ptr =
            boost::dynamic_pointer_cast<pogm::NDTMapParam>(mapParPtr))
        ndt_map_ptr->resolution_ = resolution;

    else if (pogm::NDTDLMapParamPtr ndt_map_ptr =
                 boost::dynamic_pointer_cast<pogm::NDTDLMapParam>(mapParPtr))
        ndt_map_ptr->resolutions_ = resolutions;

    cout << "finished reg parameters" << endl;

    // Check if all iputs are assigned
    if (!vm.count("base-name") || !vm.count("dir-name"))
    {
        std::cout << "Missing base or dir names.\n";
        std::cout << desc << "\n";
        return false;
    }
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return false;
    }

    use_pointtype_xyzir = vm.count("use-pointtype-xyzir");
    generate_eval_files = vm.count("generate-eval-files");
    use_only_static_scans = vm.count("use-only-static-scans");

    save_used_merged_clouds = vm.count("save-used-merged-clouds");
    save_raw_clouds = vm.count("save-raw-clouds");
    maptype_cloud = vm.count("maptype-cloud");
    save_graph_cloud = vm.count("save-graph-cloud");
    cout << "base-name:" << base_name << endl;
    cout << "dir-name:" << map_dir_name << endl;

    return true;
}
void initializeRosPublishers()
{
    ros::Time::init();
    srand(time(NULL));
    gt_pub = new ros::Publisher();
    fuser_pub = new ros::Publisher();
    odom_pub = new ros::Publisher();
    cloud_pub = new ros::Publisher();
    cloud_segmented_pub = new ros::Publisher();
    client = new ros::ServiceClient();
    *client =
        n_->serviceClient<graph_map_custom_msgs_lamide::NNDataSaver>("/INSERT_SAVE_SERVICE_TOPIC");
    *gt_pub = n_->advertise<nav_msgs::Odometry>("/GT", 50);
    *fuser_pub = n_->advertise<nav_msgs::Odometry>("/fuser", 50);
    *odom_pub = n_->advertise<nav_msgs::Odometry>("/odom", 50);
    *cloud_pub = n_->advertise<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>>("/points2", 1);
    *cloud_segmented_pub = n_->advertise<pcl::PointCloud<pcl::PointXYZL>>("/cloud_segmented", 1);
}
void printParameters()
{
    std::cout << "Output directory: " << output_dir_name << endl;
    if (filter_cloud)
    {
        std::cout << "Filtering FOV of sensor to min/max " << hori_min << " " << hori_max << endl;
        std::cout << "Filtering height of sensor to min/max " << z_min << " " << z_max << endl;
    }
    else
        std::cout << "No FOV filter." << endl;
}
void SaveMap()
{
    if (fuser_ != NULL && fuser_->FramesProcessed() > 0)
    {

        // char path[1000];
        // snprintf(path, 999, "%s/%s.map", output_dir_name.c_str(), base_name.c_str());
        if (save_map)
        {
            fuser_->SaveGraphMap(output_dir_name, base_name + ".map");
            // fuser_->SaveCurrentNodeAsJFF(path);
        }
        if (save_graph_cloud)
            fuser_->SavePointCloud(output_dir_name, base_name + ".map");
    }
}
template <class PointT>
void SegmentGroundAndPublishCloud(const pcl::PointCloud<PointT>& cloud,
                                  const Eigen::Affine3d& pose_est,
                                  pcl::PointCloud<pcl::PointXYZL>& output)
{
}
template <>
void SegmentGroundAndPublishCloud(const pcl::PointCloud<pcl::PointXYZL>& cloud,
                                  const Eigen::Affine3d& pose_est,
                                  pcl::PointCloud<pcl::PointXYZL>& output)
{ // pose est is given in a fixed frame //cloud is given in sensor frame
    pcl::PointCloud<pcl::PointXYZL> cloud_transformed = cloud;
    Eigen::Affine3d tmp = pose_est;
    perception_oru::transformPointCloudInPlace(tmp, cloud_transformed);
    // static ndt_generic::PointCloudQueue<PointT> points_filtered(10);
    Eigen::Affine3d robot_pose = pose_est * Tsensor_offset.inverse();
    // th_segment //with respect to the robot position. 0 hight of odometry frame
    double wheel_radius = 0.12;
    output.clear();
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud_transformed[i].z > robot_pose.translation()(2) - wheel_radius + th_segment)
            output.push_back(cloud_transformed[i]);
    }
    output.header.stamp = cloud.header.stamp;
    output.header.frame_id = "/world";
    if (visualize)
        cloud_segmented_pub->publish(output);
}
template <class PointT>
void PlotAll(bool registration_update,
             pcl::PointCloud<PointT>& points2,
             ndt_generic::PointCloudQueue<PointT>& queue)
{
    static tf::TransformBroadcaster br;
    ros::Time t = ros::Time::now();
    // Plot Pose estimate

    tf::Transform tf_fuser_pose;
    tf::poseEigenToTF(fuser_pose, tf_fuser_pose);
    br.sendTransform(tf::StampedTransform(tf_fuser_pose, t, "/world", "/fuser_base_link"));

    if (use_odometry_source && visualize)
    {
        Eigen::Affine3d Eig_odom_pose =
            offset * Todom_t0.inverse() *
            Todom_base; // Tgt_t0*ndt_generic::xyzrpyToAffine3d(init(0),init(1),init(2),init(3),init(4),init(5))*Todom_t0.inverse()*Todom_base;
        tf::Transform tf_odom_pose;
        tf::poseEigenToTF(Eig_odom_pose, tf_odom_pose);
        br.sendTransform(tf::StampedTransform(tf_odom_pose, t, "/world", "/odom_base_link"));
        odom_pose_msg.header.stamp = t;
        tf::poseEigenToMsg(Eig_odom_pose, odom_pose_msg.pose.pose);
        odom_pub->publish(odom_pose_msg);
    }

    if (gt_base_link_id != "" && visualize)
    {
        gt_pose_msg.header.stamp = t;
        Eigen::Affine3d Eig_gt_pose = offset * Tgt_base;
        tf::poseEigenToMsg(Eig_gt_pose, gt_pose_msg.pose.pose);
        gt_pub->publish(gt_pose_msg);
        tf::Transform tf_gt_base;
        tf::poseEigenToTF(Eig_gt_pose, tf_gt_base);
        br.sendTransform(tf::StampedTransform(tf_gt_base, t, "/world", "/state_base_link"));
    }

    if ((registration_update))
    { //
        if (visualize)
        {
            br.sendTransform(tf::StampedTransform(Tsensor_offset_tf, t, "/fuser_base_link",
                                                  "/fuser_laser_link"));
            fuser_pose_msg.header.stamp = t;
            tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);
            Eigen::MatrixXd cov = predCov;
            cov.block(0, 0, 3, 3) =
                fuser_pose.rotation() * cov.block(0, 0, 3, 3) * fuser_pose.rotation().inverse();
            cov.resize(1, 36);
            for (int i = 0; i < 36; i++)
                fuser_pose_msg.pose.covariance[i] = cov.data()[i];

            fuser_pub->publish(fuser_pose_msg);

            Eigen::Affine3d tmp = fuser_pose * Tsensor_offset;
            perception_oru::transformPointCloudInPlace(tmp, points2);
            points2.header.frame_id = "/world";
            pcl_conversions::toPCL(t, points2.header.stamp);
            cloud_pub->publish(points2);
        }
        if (save_used_merged_clouds)
        {
            // cout<<"Save! size: "<<points2.size()<<endl;
            queue.Push(points2);
            queue.Save(output_dir_name + "cloud_");
        }
    }
}
template <typename PointT> void outputcloud(PointT& point)
{
}

template <> void outputcloud(velodyne_pointcloud_oru::PointXYZIR& point)
{
    cout << point.intensity << endl;
}

template <typename PointT>
bool ProcessFrameStaticGlobalMap(pcl::PointCloud<PointT>& cloud,
                                 Eigen::Affine3d& fuser_pose,
                                 Eigen::Affine3d& Tmotion,
                                 const Eigen::Affine3d& Tsensor)
{
    return fuser_->ProcessFrameStaticGlobalMap<PointT>(cloud, fuser_pose, Tmotion);
}

template <>
bool ProcessFrameStaticGlobalMap(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                                 Eigen::Affine3d& fuser_pose,
                                 Eigen::Affine3d& Tmotion,
                                 const Eigen::Affine3d& Tsensor)
{
    std::vector<pcl::PointCloud<pcl::PointXYZL>> clouds_xyz;
    pogm::segmentPointCurvature3(Tsensor, cloud, clouds_xyz);
    return fuser_->ProcessFrameStaticGlobalMap(clouds_xyz, fuser_pose, Tmotion);
}

bool PushAndSaveBirdsEye(pcl::PointCloud<pcl::PointXYZL>& cloud, const Eigen::Affine3d& T)
{

    cloud_queue_birds_view->Push(cloud);
    pcl::PointCloud<pcl::PointXYZL> aggregated_scans;
    nav_msgs::Odometry pose;

    cloud_queue_birds_view->GetCloud(aggregated_scans);
    Eigen::Affine3d Tlatest_pose = T * Tsensor_offset.inverse();
    tf::poseEigenToMsg(Tlatest_pose, pose.pose.pose);
    cout << "pose: " << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y << ", "
         << pose.pose.pose.position.z << endl;
    Eigen::Affine3d Tlocal_frame = Eigen::Affine3d::Identity();
    Tlocal_frame =
        Tlatest_pose
            .inverse(); // ndt_generic::xyzrpyToAffine3d(init[0],init[1],init[2],init[3],init[4],init[5]).inverse()*(Tlatest_scan.inverse());
    perception_oru::transformPointCloudInPlace(Tlocal_frame, aggregated_scans);
    toPCL(ros::Time::now(), aggregated_scans.header.stamp);
    aggregated_scans.header.frame_id = "/world";
    toPCL(ros::Time::now(), aggregated_scans.header.stamp);

    graph_map_custom_msgs_lamide::NNDataSaver srv;

    cout << "Request with cloud of size: " << aggregated_scans.size() << endl;
    pcl::toROSMsg(aggregated_scans, srv.request.vels);
    cloud_segmented_pub->publish(srv.request.vels);
    srv.request.pose = pose;

    if (client->call(srv))
    {
        std::cout << srv.response.status << std::endl;
        ;
        return true;
    }
    else
    {
        ROS_ERROR("Could not get a response");
        return false;
    }
}
template <typename PointT>
bool ProcessFrame(pcl::PointCloud<PointT>& cloud,
                  Eigen::Affine3d& fuser_pose,
                  Eigen::Affine3d& Tmotion,
                  Eigen::MatrixXd& Tcov,
                  const Eigen::Affine3d& Tsensor)
{
    // std::vector<pcl::PointCloud<pcl::PointXYZL> > segmented_clouds;
    // segmentPointCurvature3(Tsensor, cloud, segmented_clouds);
    if (visualize)
    {
        pogm::GraphPlot::PlotScan(cloud, std::string("fuser"), std::string("cloud"));
    }
    return fuser_->ProcessFrame<PointT>(cloud, fuser_pose, Tmotion, Tcov);
}
template <>
bool ProcessFrame(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                  Eigen::Affine3d& fuser_pose,
                  Eigen::Affine3d& Tmotion,
                  Eigen::MatrixXd& Tcov,
                  const Eigen::Affine3d& Tsensor)
{
    std::vector<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>> clouds_xyzir;
    pogm::segmentPointCurvature3(Tsensor, cloud, clouds_xyzir);
    if (visualize)
    {
        for (int i = 0; i < clouds_xyzir.size(); i++)
        {
            pogm::GraphPlot::PlotScan(clouds_xyzir[i], std::string("fuser"),
                                      std::string("cloud") + ndt_generic::toString(i));
        }
    }
    std::vector<pcl::PointCloud<pcl::PointXYZL>> clouds_xyz(clouds_xyzir.size());
    clouds_xyz.resize(clouds_xyzir.size());
    for (int i = 0; i < clouds_xyzir.size(); i++)
    {
        for (int j = 0; j < clouds_xyzir[i].size(); j++)
        {
            pcl::PointXYZL p;
            p.x = clouds_xyzir[i][j].x;
            p.y = clouds_xyzir[i][j].y;
            p.z = clouds_xyzir[i][j].z;
            clouds_xyz[i].push_back(p);
        }
    }
    return fuser_->ProcessFrame(clouds_xyz, fuser_pose, Tmotion, Tcov);
}

template <typename PointT> void processData()
{

    n_ = new ros::NodeHandle("~");
    fuser_ = NULL;
    ndt_generic::PointCloudQueue<PointT> queue(
        1); // WARN THIS NEEDS TO BE SET TO 10 FOR KEVINS SOFTWARE

    stringstream ss;
    string name = "";
    ss << name << dataset << "_gt=" << gt_mapping << std::string("_submap=") << use_submap
       << "_sizexy=" << map_size_xy << "_Z=" << map_size_z << std::string("_intrchR=")
       << interchange_radius << std::string("_compR=") << compound_radius << std::string("_res=")
       << resolution << std::string("_maxSensd=") << max_range << "_keyF=" << use_keyframe
       << "_d=" << min_keyframe_dist << "_deg=" << min_keyframe_dist_rot_deg
       << "_alpha=" << distance_alpha;
    base_name += ss.str();
    bool dl = (map_type_name == "ndt_dl_map");
    base_name += "_dl=" + toString(dl) + "_xyzir=" + toString(use_pointtype_xyzir) +
                 "_mpsu=" + toString(min_nb_points_set_uniform) +
                 "_mnpfg=" + toString(min_nb_points_for_gaussian) + "kmnp" +
                 toString(keep_min_nb_points);
    ndt_generic::CreateEvalFiles eval_files(output_dir_name, base_name, generate_eval_files);

    printParameters();
    initializeRosPublishers();

    std::string inter_base_link = base_link_id, interp_parent_id = odom_parent_link;
    if (gt_mapping)
    {
        inter_base_link = gt_base_link_id;
        interp_parent_id = odom_gt_parent_link;
    }

    std::vector<std::string> ros_bag_paths;
    cout << "look for bags at " << map_dir_name << std::endl;
    if (!ndt_generic::LocateRosBagFilePaths(map_dir_name, ros_bag_paths))
    {
        std::cout << "couldnt locate ros bags" << endl;
        exit(0);
    }
    Eigen::Vector3d prev_pos;
    counter = 0;
    std::cout << "opening bag files" << endl;
    double dist_traveled = 0;
    double height_init = 0;
    for (int i = 0; i < ros_bag_paths.size() && ros::ok(); i++)
    {

        bool initiated = false;
        std::string bagfilename = ros_bag_paths[i];
        std::cout << "Opening bag file:" << bagfilename.c_str() << endl;

        ndt_offline::readPointCloud reader(bagfilename, Tsensor_offset, odom_type, lidar_topic,
                                           min_range, max_range, velodyne_config_file,
                                           sensor_time_offset, tf_topic, interp_parent_id,
                                           inter_base_link, unpack_raw);
        ndt_generic::StepControl step_controller;
        bool found_scan = true;
        Tgt_base = Todom_base = Eigen::Affine3d::Identity();
        Tgt_t0 = Eigen::Affine3d::Identity();
        while (found_scan && n_->ok())
        {
            pcl::PointCloud<PointT> cloud_raw, cloud, points2;

            ros::Time t1 = ros::Time::now();
            found_scan = reader.readNextMeasurement<PointT>(cloud_raw);
            cout << "Read: " << cloud_raw.size() << endl;
            if (cloud_raw.size() == 0)
            {
                cout << "Empty cloud" << endl;
                continue;
            }
            else if (!found_scan)
                break;
            ros::Time t2 = ros::Time::now();

            if (save_raw_clouds)
            {
                pcl::io::savePCDFileBinary(output_dir_name + "raw_cloud" +
                                               ndt_generic::toString(counter) + ".pcd",
                                           cloud_raw);
            }

            if (filter_cloud)
            {
                ndt_generic::filter_range_fun(cloud, cloud_raw, min_range, max_range);
                ndt_generic::filter_height_angle(cloud, hori_min, hori_max, z_min, z_max);
            }
            else
                cloud = cloud_raw;

            if (cloud.size() == 0)
                continue; // Check that we have something to work with depending on the FOV filter
                          // here...

            bool odometry_valid = true, gt_valid = true;
            if (use_odometry_source)
                odometry_valid = reader.GetOdomPose(reader.GetTimeOfLastCloud(), base_link_id,
                                                    odom_parent_link, Todom_base);
            if (use_gt_data)
                gt_valid = reader.GetOdomPose(reader.GetTimeOfLastCloud(), gt_base_link_id,
                                              odom_gt_parent_link, Tgt_base);

            if ((use_gt_data && !gt_valid) || (use_odometry_source && !odometry_valid))
            {
                cout << "Invalid odometry, skipping frame" << endl;
                counter++;
                continue;
            }
            if (fuser_ == NULL)
            {

                if (use_gt_data)
                {
                    Tgt_base_prev = Tgt_base;
                    Tgt_t0 = Tgt_base;
                    if (use_odometry_source)
                        Todom_t0 = Todom_base;
                }
                if (use_odometry_source)
                    Todom_base_prev = Todom_base;
                offset = ndt_generic::xyzrpyToAffine3d(init(0), init(1), init(2), init(3), init(4),
                                                       init(5));
                cout << "offset: " << offset.rotation().eulerAngles(0, 1, 2) << endl;
                fuser_pose_init = ndt_generic::xyzrpyToAffine3d(init(0), init(1), init(2), init(3),
                                                                init(4), init(5)) *
                                  Tgt_t0; // specify initial pose offset with respect to GT, (if
                                          // avaliable), otherwise Identity.
                fuser_pose = fuser_pose_init;
                height_init = fuser_pose.translation()(2);
                prev_pos = fuser_pose.translation();

                if (init_map_gt_frame)
                    fuser_ = new pogm::GraphMapFuser(regParPtr, mapParPtr, graphParPtr,
                                                     Eigen::Affine3d::Identity(), Tsensor_offset,
                                                     output_dir_name);
                else
                {
                    cout << "init at fuser: " << fuser_pose.translation().transpose() << endl;
                    fuser_ = new pogm::GraphMapFuser(regParPtr, mapParPtr, graphParPtr,
                                                     fuser_pose * Tsensor_offset, Tsensor_offset,
                                                     output_dir_name);
                }

                fuser_->SetFuserOptions(save_used_merged_clouds);
                fuser_->SetkeyframeOptions(use_only_static_scans);
                fuser_->SetMotionModel(mot_model);
                pogm::GraphMapNavigatorPtr graph_map = fuser_->GetGraph();
                vis = new pogm::graphVisualization(graph_map, visualize, visualize_map, true);
                vis->SetParameters(Tmap, Tother);

                std::cout << fuser_->ToString() << endl;
                initiated = true;
            }
            else if (!initiated && fuser_ != NULL)
            { // new bag file

                // fuser_pose = offset*Tgt_base; //WARNING SET TO GT EVERY NEW BAG
                // Todom_base_prev = Todom_base;
                // Tgt_base_prev = Tgt_base;
                initiated = true;
            }

            predCov = Eigen::MatrixXd::Identity(6, 6);
            bool registration_update = true;
            Eigen::Affine3d Tmotion = Eigen::Affine3d::Identity();

            if (gt_mapping)
            {
                Tmotion = Eigen::Affine3d::Identity();
                fuser_pose = offset * Tgt_base;

                for (int i = 0; i < 6; i++)
                    predCov(i, i) = sigma(i);
                predCov = predCov / lambda;
            }
            else if (use_odometry_source)
            {
                Tmotion = Todom_base_prev.inverse() * Todom_base;
                predCov =
                    fuser_->PredictOdomUncertainty(fuser_pose * Tmotion, registration2d) / lambda;
            }
            perception_oru::transformPointCloudInPlace(Tsensor_offset, cloud);

            if (!gt_mapping)
            {
                if (use_only_static_scans)
                    registration_update = ProcessFrameStaticGlobalMap<PointT>(
                        cloud, fuser_pose, Tmotion, Tsensor_offset);
                else
                    registration_update =
                        ProcessFrame<PointT>(cloud, fuser_pose, Tmotion, predCov, Tsensor_offset);
            }

            if (save_birds_eye && registration_update)
            {
                pcl::PointCloud<pcl::PointXYZL> filtered;
                SegmentGroundAndPublishCloud(cloud, fuser_pose, filtered);
                pcl::PointCloud<pcl::PointXYZL> cloud_filtered_xyz;
                PushAndSaveBirdsEye(filtered, fuser_pose * Tsensor_offset);
            }

            ros::Time t3 = ros::Time::now();
            counter++;
            PlotAll<PointT>(registration_update, cloud_raw, queue);

            if (registration_update)
            {
                fuser_->AddPoseFramePair(fuser_pose * Tsensor_offset, Tgt_base * Tsensor_offset);
                eval_files.Write(reader.GetTimeOfLastCloud(), offset * Tgt_base,
                                 offset * Todom_base, fuser_pose, fuser_pose * Tsensor_offset);
                prev_pos = fuser_pose.translation();
            }
            // for(int i=0;i<cloud.size();i+=100){
            //   outputcloud(cloud.points[i]);
            // }

            ros::Time t4 = ros::Time::now();
            Tgt_base_prev = Tgt_base;
            Todom_base_prev = Todom_base;
            cout << "read bag= " << t2 - t1 << ", process frame= " << t3 - t2
                 << ", plot=" << t4 - t3 << "size: " << cloud.size() << endl;

            if (registration_update && step_control)
                step_controller.Step(counter);
        }
    }

    eval_files.Close();
    vis->~graphVisualization();
    SaveMap();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_fuser3d_offline");
    bopo::options_description desc("Allowed options");

    std::cout << "Read params" << endl;
    bool succesfull = ReadAllParameters(desc, argc, &argv);
    if (!succesfull)
        exit(0);

    if (use_pointtype_xyzir)
    {
        cout << "velodyne_pointcloud_oru::PointXYZIR" << endl;
        processData<velodyne_pointcloud_oru::PointXYZIR>();
    }
    else
    {
        cout << "pcl::PointXYZL" << endl;
        processData<pcl::PointXYZL>();
    }

    if (alive)
    {
        while (1)
        {
            usleep(1000);
        }
    }
    usleep(1000 * 1000);
    std::cout << "Done." << std::endl;
}
