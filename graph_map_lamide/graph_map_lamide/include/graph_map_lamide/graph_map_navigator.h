#ifndef GRAPH_NAVIGATOR_INTERFACE_H
#define GRAPH_NAVIGATOR_INTERFACE_H
#include "Eigen/Dense"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/serialization.hpp"
#include "boost/serialization/shared_ptr.hpp"
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/map_node.h"
#include "graph_map_lamide/map_type.h"
#include "graphfactory.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "reg_type.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>

namespace perception_oru
{
namespace graph_map
{

typedef enum SelectMap
{
    node_position = 0,
    mean_observation = 1,
    closest_observation = 2,
    grid = 3,
    node_position_esg = 4,
    overlap = 5,
    overlap_registration = 6
} MapSwitchingMethod;

class GraphMapNavigator : public GraphMap
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphMapNavigator(const Eigen::Affine3d& nodepose,
                      const MapParamPtr& mapparam,
                      const GraphMapParamPtr graphparam);

    GraphMapNavigator(const GraphMapParamPtr graphparam);

    GraphMapNavigator()
        : GraphMap()
    {
    }

    std::string ToString();

    //!
    //! \brief GetMapByOverlap computes the closest map depeing on hte overlap between current scan
    //! and a map in vicinity of the sensor \param Tnow the pose of the sensor \return pointer to
    //! closest map
    //!
    MapNodePtr GetMapByOverlap(const Eigen::Affine3d& Tnow,
                               pcl::PointCloud<pcl::PointXYZL>& cloud,
                               int count = -1);

    MapNodePtr GetMapNodeByObservationHistory(const Eigen::Affine3d& Tnow);

    MapNodePtr GetMapNodeByObservationHistory(const Eigen::Affine3d& Tnow, unsigned int n_search);

    MapNodePtr GetMapByRegistration(Eigen::Affine3d& Tnow,
                                    pcl::PointCloud<pcl::PointXYZL>& cloud,
                                    RegTypePtr& regPtr);

    std::vector<MapNodePtr> GetClosestNodes(
        const Eigen::Affine3d& Tnow,
        double max_distance,
        bool factor_interchange = false); // if factor_interchange==true, max distance equals
                                          // interchange_radius*max_distance

    void UpdateGraph(const Eigen::Affine3d& pose, pcl::PointCloud<pcl::PointXYZL>& cloud);

    void WorldToLocalMapFrame(Eigen::Affine3d& pose, MapNodePtr frame = NULL);
    void LocalToWorldMapFrame(Eigen::Affine3d& pose, MapNodePtr frame = NULL);

    bool SwitchToClosestMapNode(const Affine3d& Tnow, double max_distance);
    //!
    //! \brief SwitchToClosestMapNode Attempts to find the closest Mapnode within desired radius
    //! \param Tnow is the position around the closest map will be found
    //! \param cov not used5
    //! \param T_world_to_local_map not used
    //! \param radius the maximum distance to search within, 0.0 allows nodes at any distance
    //! \return true if a node was found within range

    bool SwitchToClosestMapNode(const Affine3d& Tnow,
                                pcl::PointCloud<pcl::PointXYZL>& cloud,
                                double max_distance,
                                int count = -1);
    // bool SwitchToClosestMapNode(const Eigen::Affine3d &Tnow, const Matrix6d &cov, const double
    // radius);
    //!
    //! \brief SwitchToClosestMapNode returns the closest map node, return true if a node was fund
    //! \param Tnow is the target pose in the global frame
    //! \param use_map_centroid
    //! \return true if a node was found
    //!

    void SetMapSwitchMethod(MapSwitchingMethod map_switch_method = node_position)
    {
        std::cout << "set map switch method to " << map_switch_method << std::endl;
        map_switch_method_ = map_switch_method;
    }

    void SetMapInterchangeRadius(double radius)
    {
        std::cout << "set SetMapInterchangeRadius to " << radius << std::endl;
        interchange_radius_ = radius;
    }

    MapSwitchingMethod GetMapSwitchMethod()
    {
        return map_switch_method_;
    }

    bool AutomaticMapInterchange(const Eigen::Affine3d& Tnow,
                                 const Matrix6d& cov,
                                 bool& changed_map_node,
                                 bool& created_map_node);

    void AddMapNode(const Eigen::Affine3d& diff, const Matrix6d& cov = unit_covar)
    {
        GraphMap::AddMapNode(diff, cov);
    } // map_nodes_grid_.SetVal(currentNode_,GetCurrentNodePose().translation());

    bool SwitchToMapNode(MapNodePtr new_node);

    const Eigen::Affine3d& GetSensorPose() const
    {
        return Tsensor_;
    }

    Eigen::Affine3d& GetSensorPose()
    {
        return Tsensor_;
    }

    void getNodeStates();

    void unloadAllSubmaps();

    bool getCheckForBrokenSubmaps() const
    {
        return check_for_broken_submaps_;
    }

    void setCheckForBrokenSubmaps(bool check)
    {
        check_for_broken_submaps_ = check;
    }

    void setMapParams(const MapParamPtr& mapparam)
    {
        mapparam_ = mapparam;
    }

protected:
    bool use_submap_ = false;
    bool check_for_broken_submaps_ = false;
    double interchange_radius_ = 1000;
    double compound_radius_ = 0;
    bool use_keyframe_ = true;
    double min_keyframe_dist_ = 0.5;
    double min_keyframe_rot_deg_ = 15;

    MapSwitchingMethod map_switch_method_ = node_position;
    unsigned int n_search_ = 15;
    Eigen::Affine3d Tsensor_ = Eigen::Affine3d::Identity();
    std::map<int, int> visited_nodes_;
    int previous_map_id_ = -1;

private:
    MapNodePtr GetClosestMapNode(const Eigen::Affine3d& Tnow,
                                 const bool use_observation_centroid = false);

    bool TransitionSG(const Eigen::Affine3d& Tnow,
                      const Matrix6d& cov_incr,
                      bool& changed_map_node,
                      bool& created_map_node);

    bool TransitionESG(const Eigen::Affine3d& Tnow,
                       const Matrix6d& cov_incr,
                       bool& changed_map_node,
                       bool& created_map_node);

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(
        Archive& ar,
        const unsigned int version) // In order to clal this you need to register it to boost using
                                    // "ar.template register_type<LazyGrid>();"
    {
        ar& boost::serialization::base_object<GraphMap>(*this);
        ar& use_submap_;
        ar& interchange_radius_;
        ar& compound_radius_;
        ar& use_keyframe_;
        ar& min_keyframe_dist_;
        ar& min_keyframe_rot_deg_;
        // ar & map_nodes_grid_;
        ar& Tsensor_;
    }
};

bool LoadGraphMap(const std::string& file_path,
                  const std::string& file_name,
                  GraphMapNavigatorPtr& ptr,
                  bool check_for_broken_submaps = false);

void SaveGraphMap(const std::string& path,
                  const std::string& filename,
                  GraphMapNavigatorPtr graph_map);

void SaveObservationVector(const std::string& file_name, GraphMapNavigatorPtr graph_map);

class GraphMapNavigatorParam : public GraphMapParam
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphMapNavigatorParam()
    {
    }
    /*!
     * \brief ParseSetMapSwitchMethod
     * \param switch_method "mean_observation" "closest_observation" or default("node_position")
     * \return
     */
    static MapSwitchingMethod String2SwitchMethod(const std::string& switch_method);

    static std::string SwitchMethod2String(const MapSwitchingMethod& switch_method);

    void GetParametersFromRos();
    bool use_submap = false;
    bool check_for_broken_submaps = false;
    bool use_keyframe = true;
    double interchange_radius = 1000;
    double compound_radius = 0;
    double min_keyframe_dist = 0.5;
    double min_keyframe_rot_deg = 15;
    double alpha = 0.0; // use angular distance
    unsigned int n_search = 5;
    Eigen::Affine3d Tsensor = Eigen::Affine3d::Identity();

    MapSwitchingMethod map_switch_method = node_position;

private:
    friend class GraphFactory;
};

} // namespace graph_map
} // namespace perception_oru
#endif // GRAPH_NAVIGATOR_INTERFACE_H
