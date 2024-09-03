#ifndef GRAPH_PLOT_H
#define GRAPH_PLOT_H
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/PoseArray.h"
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/octomap/octomap_map_type.h"
#include "nav_msgs/Odometry.h"
#include "ndt_generic_lamide/point_types.h"
#include "ndt_localization_lamide/particle.hpp"
#include "ndt_map_lamide/NDTMapMsg.h"
#include "ndt_map_lamide/ndt_cell.h"
#include "ndt_map_lamide/ndt_conversions.h"
#include "ndt_map_lamide/ndt_map.h"
#include "octomap/octomap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"


#define NDT_GLOBAL_MAP_TOPIC     "NDTglobalMap"
#define NDT_OM_MAP_TOPIC         "NDTOmMap"
#define OCCUPANCY_TOPIC          "occupancy"
#define NDT_GLOBAL2_MAP_TOPIC    "NDTglobal2Map"
#define NDT_LOCAL_MAP_TOPIC      "NDTlocalMap"
#define GRAPH_POSE_TOPIC         "graphMap"
#define GRAPH_INFO_TOPIC         "graphInfo"
#define PARTICLES_TOPIC          "ParticleCloud"
#define OBSERVATIONS_TOPIC       "observations"
#define TRAJECTORY_TOPIC         "Trajectory"
#define EXT_TRAJECTORY_TOPIC     "gt_trajectory"
#define EST_TRAJECTORY_TOPIC     "est_trajectory"
#define CURRENT_TRAJECTORY_TOPIC "current_submap_source_locations"
#define CLOUD_MAP                "cloud_map"
#define CLOUD_CURRENT            "cloud_current"
#define CLOUD_PREV               "cloud_previous"
namespace perception_oru
{
namespace graph_map
{
using Eigen::Affine3d;
using std::cerr;
using std::cout;
using std::endl;
typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> cov_vector;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mean_vector;
typedef enum plotmarker
{
    sphere = 0,
    cross = 1,
    point = 2,
    occupancy = 3
} PlotMarker;
class GraphPlot
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static void sendMapToRviz(mean_vector& mean,
                              cov_vector& cov,
                              ros::Publisher* mapPublisher,
                              string frame,
                              int color,
                              const Affine3d& offset = Affine3d::Identity(),
                              string ns = "ndt",
                              int markerType = visualization_msgs::Marker::SPHERE);

    static void PlotScan(pcl::PointCloud<pcl::PointXYZL>& cloud,
                         const std::string& frame_id,
                         const std::string& topic,
                         const Affine3d& offset = Affine3d::Identity());

    static void PlotScan(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                         const std::string& frame_id,
                         const std::string& topic,
                         const Affine3d& offset = Affine3d::Identity());

    static void PlotNDT(pcl::PointCloud<pcl::PointXYZL>& cloud,
                        const std::string& frame_id,
                        const std::string& topic_scan,
                        const Affine3d& offset = Affine3d::Identity());

    static void PlotNDT(NDTMap*& ndt,
                        const std::string& frame_id,
                        const std::string& topic_scan,
                        const Affine3d& offset = Affine3d::Identity());

    static void PlotNDTRGB(NDTMap*& ndt,
                        const std::string& frame_id,
                        const std::string& topic_scan,
                        const Affine3d& offset = Affine3d::Identity(),
                        int colors = 1);



    static void PlotNDT(std::vector<perception_oru::NDTCell*>& ndt,
                        const std::string& frame_id,
                        const std::string& topic);

    // static void SendGlobalMapToRviz(perception_oru::NDTMap *mapPtr, int color,const Affine3d
    // &offset=Affine3d::Identity()); static void SendGlobal2MapToRviz(perception_oru::NDTMap
    // *mapPtr, int color,const Affine3d &offset=Affine3d::Identity()); static void
    // SendGlobal2MapToRviz(std::vector<perception_oru::NDTCell*>cells, int color,const Affine3d
    // &offset=Affine3d::Identity());
    static void plotParticleCloud(const Eigen::Affine3d& offset, std::vector<particle> pcloud);
    // static void PlotPoseGraph(GraphMapPtr graph);
    static void PlotMap(MapTypePtr map,
                        int color,
                        const Affine3d& offset = Affine3d::Identity(),
                        PlotMarker marker = sphere,
                        std::string ns = "");
    static void PlotMap(NDTMap& map,
                        int color,
                        const Affine3d& offset = Affine3d::Identity(),
                        PlotMarker marker = sphere,
                        std::string ns = "ndtmap",
                        double point_size = 0.1);
    static void PlotObservationVector(GraphMapPtr graph_map);
    // static void PublishOctreeMap(OctomapMapTypePtr octmap);
private:
    static void PublishMapAsOccupancy(mean_vector& mean,
                                      const std::vector<double>& occupancy,
                                      double scale,
                                      const Eigen::Affine3d& offset,
                                      std::string ns,
                                      int id = 0);
    static void PublishMapAsPoints(mean_vector& mean,
                                   int color,
                                   double scale,
                                   const Eigen::Affine3d& offset,
                                   std::string ns = "pts",
                                   int id = 0);
    static void CovarToMarker(const Eigen::Matrix3d& cov,
                              const Eigen::Vector3d& mean,
                              visualization_msgs::Marker& marker);
    static void GetAllCellsOccupancy(std::vector<perception_oru::NDTCell*> cells,
                                     mean_vector& center_position,
                                     std::vector<double>& occupancy);
    static void GetAllCellsMeanCov(const perception_oru::NDTMap* mapPtr,
                                   cov_vector& cov,
                                   mean_vector& mean,
                                   std::vector<double>& occ);
    static void GetAllCellsMeanCov(std::vector<perception_oru::NDTCell*> cells,
                                   cov_vector& cov,
                                   mean_vector& mean,
                                   std::vector<double>& occupancy);
    static void makeRightHanded(Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues);
    static void computeShapeScaleAndOrientation3D(const Eigen::Matrix3d& covariance,
                                                  Eigen::Vector3d& scale,
                                                  Eigen::Quaterniond& orientation);
    static void Initialize();
    static bool FindPublisherHandle(const std::string& topic, ros::Publisher*& pub);
    static bool initialized_;
    static ros::NodeHandle* nh_;
    static ros::Publisher* localMapPublisher_;
    static ros::Publisher *globalMapPublisher_, *global2MapPublisher_;
    static ros::Publisher *graphPosePublisher_, *graphInfoPublisher_, *particlaCloudPublisher_,
        *observationPublisher_, *trajectoryPublisher_;
    static ros::Publisher* map_pub_;
    static std::vector<std::pair<std::string, ros::Publisher*>> pub_handles;

private:
    GraphPlot();
    class ColorGradient
    {
    private:
        struct ColorPoint // Internal class used to store colors at different points in the
                          // gradient.
        {
            float r, g, b; // Red, green and blue values of our color.
            float val;     // Position of our color along the gradient (between 0 and 1).
            ColorPoint(float red, float green, float blue, float value)
            {
                if (red > 1.0001)
                    r = red / 255.0;
                else
                    r = red;

                if (green > 1.0001)
                    g = green / 255.0;
                else
                    g = green;

                if (blue > 1.0001)
                    b = blue / 255.0;
                else
                    b = blue;

                val = value;
            }
        };
        std::vector<ColorPoint> color; // An array of color points in ascending value.

    public:
        //-- Default constructor:
        ColorGradient()
        {
            createDefaultHeatMapGradient();
        }

        //-- Inserts a new color point into its correct position:
        void addColorPoint(float red, float green, float blue, float value)
        {

            for (int i = 0; i < color.size(); i++)
            {
                if (value < color[i].val)
                {
                    color.insert(color.begin() + i, ColorPoint(red, green, blue, value));
                    return;
                }
            }
            color.push_back(ColorPoint(red, green, blue, value));
        }

        //-- Inserts a new color point into its correct position:
        void clearGradient()
        {
            color.clear();
        }

        //-- Places a 5 color heapmap gradient into the "color" vector:
        void createDefaultHeatMapGradient()
        {
            color.clear();
            color.push_back(ColorPoint(0, 0, 1, 0.0f));  // Blue.
            color.push_back(ColorPoint(0, 1, 1, 0.25f)); // Cyan.
            color.push_back(ColorPoint(0, 1, 0, 0.5f));  // Green.
            color.push_back(ColorPoint(1, 1, 0, 0.75f)); // Yellow.
            color.push_back(ColorPoint(1, 0, 0, 1.0f));  // Red.
        }
        void CreateVisibleSpectrum()
        {
            color.clear();
            color.push_back(ColorPoint(1, 0, 1, 0.0f));  // Blue.
            color.push_back(ColorPoint(0, 0, 1, 0.25f)); // Cyan.
            color.push_back(ColorPoint(0, 1, 0, 0.5f));  // Green.
            color.push_back(ColorPoint(1, 1, 0, 0.75f)); // Yellow.
            color.push_back(ColorPoint(1, 0, 0, 1.0f));  // Red.
        }
        void CreateMaxDistinctColors()
        {
            color.push_back(ColorPoint(0, 0, 0, 0.0f));
            color.push_back(ColorPoint(1, 0, 103, 0.015625f));
            color.push_back(ColorPoint(213, 255, 0, 0.03125f));
            color.push_back(ColorPoint(255, 0, 86, 0.046875f));
            color.push_back(ColorPoint(158, 0, 142, 0.0625f));
            color.push_back(ColorPoint(14, 76, 161, 0.078125f));
            color.push_back(ColorPoint(255, 229, 2, 0.09375f));
            color.push_back(ColorPoint(0, 95, 57, 0.10937500f));
            color.push_back(ColorPoint(0, 255, 0, 0.1250f));
            color.push_back(ColorPoint(149, 0, 58, 0.14062500f));
            color.push_back(ColorPoint(255, 147, 126, 0.15625f));
            color.push_back(ColorPoint(164, 36, 0, 0.17187500f));
            color.push_back(ColorPoint(0, 21, 68, 0.1875f));
            color.push_back(ColorPoint(145, 208, 203, 0.2031250f));
            color.push_back(ColorPoint(98, 14, 0, 0.21875f));
            color.push_back(ColorPoint(107, 104, 130, 0.2343750f));
            color.push_back(ColorPoint(0, 0, 255, 0.2500f));
            color.push_back(ColorPoint(0, 125, 181, 0.26562500f));
            color.push_back(ColorPoint(106, 130, 108, 0.28125f));
            color.push_back(ColorPoint(0, 174, 126, 0.29687500f));
            color.push_back(ColorPoint(194, 140, 159, 0.3125f));
            color.push_back(ColorPoint(190, 153, 112, 0.32812500f));
            color.push_back(ColorPoint(0, 143, 156, 0.34375f));
            color.push_back(ColorPoint(95, 173, 78, 0.35937500f));
            color.push_back(ColorPoint(255, 0, 0, 0.3750f));
            color.push_back(ColorPoint(255, 0, 246, 0.39062500f));
            color.push_back(ColorPoint(255, 2, 157, 0.40625f));
            color.push_back(ColorPoint(104, 61, 59, 0.42187500f));
            color.push_back(ColorPoint(255, 116, 163, 0.4375f));
            color.push_back(ColorPoint(150, 138, 232, 0.45312500f));
            color.push_back(ColorPoint(152, 255, 82, 0.46875f));
            color.push_back(ColorPoint(167, 87, 64, 0.48437500f));
            color.push_back(ColorPoint(1, 255, 254, 0.5f));
            color.push_back(ColorPoint(255, 238, 232, 0.51562500f));
            color.push_back(ColorPoint(254, 137, 0, 0.53125f));
            color.push_back(ColorPoint(189, 198, 255, 0.54687500f));
            color.push_back(ColorPoint(1, 208, 255, 0.5625f));
            color.push_back(ColorPoint(117, 68, 177, 0.57812500f));
            color.push_back(ColorPoint(165, 255, 210, 0.59375f));
            color.push_back(ColorPoint(255, 166, 254, 0.60937500f));
            color.push_back(ColorPoint(119, 77, 0, 0.6250f));
            color.push_back(ColorPoint(122, 71, 130, 0.64062500f));
            color.push_back(ColorPoint(38, 52, 0, 0.65625f));
            color.push_back(ColorPoint(0, 71, 84, 0.67187500f));
            color.push_back(ColorPoint(67, 0, 44, 0.6875f));
            color.push_back(ColorPoint(181, 0, 255, 0.70312500f));
            color.push_back(ColorPoint(255, 177, 103, 0.71875f));
            color.push_back(ColorPoint(255, 219, 102, 0.73437500f));
            color.push_back(ColorPoint(144, 251, 146, 0.7500f));
            color.push_back(ColorPoint(126, 45, 210, 0.76562500f));
            color.push_back(ColorPoint(189, 211, 147, 0.78125f));
            color.push_back(ColorPoint(229, 111, 254, 0.79687500f));
            color.push_back(ColorPoint(222, 255, 116, 0.8125f));
            color.push_back(ColorPoint(0, 255, 120, 0.82812500f));
            color.push_back(ColorPoint(0, 155, 255, 0.84375f));
            color.push_back(ColorPoint(0, 100, 1, 0.85937500f));
            color.push_back(ColorPoint(0, 118, 255, 0.8750f));
            color.push_back(ColorPoint(133, 169, 0, 0.89062500f));
            color.push_back(ColorPoint(0, 185, 23, 0.90625f));
            color.push_back(ColorPoint(120, 130, 49, 0.92187500f));
            color.push_back(ColorPoint(0, 255, 198, 0.9375f));
            color.push_back(ColorPoint(255, 110, 65, 0.95312500f));
            color.push_back(ColorPoint(232, 94, 190, 0.96875f));
        }

        //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
        //-- values representing that position in the gradient.
        void getColorAtValue(const float value, float& red, float& green, float& blue)
        {
            if (color.size() == 0)
                return;

            for (int i = 0; i < color.size(); i++)
            {
                ColorPoint& currC = color[i];
                if (value < currC.val)
                {
                    ColorPoint& prevC = color[std::max(0, i - 1)];
                    float valueDiff = (prevC.val - currC.val);
                    float fractBetween = (valueDiff == 0) ? 0 : (value - currC.val) / valueDiff;
                    red = (prevC.r - currC.r) * fractBetween + currC.r;
                    green = (prevC.g - currC.g) * fractBetween + currC.g;
                    blue = (prevC.b - currC.b) * fractBetween + currC.b;
                    return;
                }
            }
            red = color.back().r;
            green = color.back().g;
            blue = color.back().b;
            return;
        }
    };
};

} // namespace graph_map
} // namespace perception_oru
#endif // GRAPH_PLOT_H
