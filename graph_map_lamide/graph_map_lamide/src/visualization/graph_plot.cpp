#include "graph_map_lamide/visualization/graph_plot.h"

#include "graph_map_lamide/graph_map_navigator.h"

#include <ndt_rviz_lamide/ndt_rviz.h>


namespace perception_oru
{
namespace graph_map
{
bool GraphPlot::initialized_ = false;
ros::NodeHandle* GraphPlot::nh_ = NULL;
ros::Publisher* GraphPlot::localMapPublisher_ = NULL;
ros::Publisher* GraphPlot::graphPosePublisher_ = NULL;
ros::Publisher* GraphPlot::graphInfoPublisher_ = NULL;
ros::Publisher* GraphPlot::globalMapPublisher_ = NULL;
ros::Publisher* GraphPlot::observationPublisher_ = NULL;
ros::Publisher* GraphPlot::global2MapPublisher_ = NULL;
ros::Publisher* GraphPlot::particlaCloudPublisher_ = NULL;
ros::Publisher* GraphPlot::trajectoryPublisher_ = NULL;
ros::Publisher* GraphPlot::map_pub_ = NULL;
std::vector<std::pair<std::string, ros::Publisher*>> GraphPlot::pub_handles;

void GraphPlot::PlotMap(
    MapTypePtr map, int color, const Affine3d& offset, PlotMarker marker, std::string ns)
{

    if (NDTMapPtr ptr = boost::dynamic_pointer_cast<NDTMapType>(map))
    {

        if (marker == plotmarker::sphere)
        {
        }
        else if (marker == plotmarker::point)
        {
            cov_vector cov;
            mean_vector mean;
            std::vector<double> occ;
            GetAllCellsMeanCov(ptr->GetNDTMap(), cov, mean, occ);
            PublishMapAsPoints(mean, color, 0.2 * ptr->GetResolution(), offset, ns);
        }
        else if (marker == plotmarker::occupancy)
        {
            cov_vector cov;
            mean_vector center;
            std::vector<double> occ;
            NDTMap* map = ptr->GetNDTMap();
            std::vector<NDTCell*> cells = map->getAllCells();
            GetAllCellsOccupancy(cells, center, occ);
            PublishMapAsOccupancy(center, occ, 1.0 * ptr->GetResolution(), offset, ns);
        }
    }
    /*else if(OctomapMapTypePtr ptr=boost::dynamic_pointer_cast<OctomapMapType>(map)){
      if(marker==plotmarker::point){
        cov_vector cov;
        mean_vector mean;
        std::vector<double> occ;
        //PublishMapAsPoints(mean,color,0.2*ptr->GetResolution(),offset, ns);
      }

    }
    if(NDTDLMapPtr ptr=boost::dynamic_pointer_cast<NDTDLMapType>(map)){
      if(marker==plotmarker::sphere){
        GraphPlot::SendGlobalMapToRviz( ptr->GetNDTMapFlat(),color,offset);
        //      GraphPlot::SendGlobalMapToRviz( ptr->GetNDTMapEdge(),color+1,offset);
      }
      else if(marker==plotmarker::point){
        cov_vector cov;
        mean_vector mean;
        std::vector<double> occ;
        GetAllCellsMeanCov(ptr->GetNDTMapFlat(), cov, mean,occ);
        PublishMapAsPoints(mean,color,0.4*ptr->GetResolution(),offset, ns+std::string("flat"));
        GetAllCellsMeanCov(ptr->GetNDTMapEdge(), cov, mean,occ);
        PublishMapAsPoints(mean,color+2,ptr->GetResolution(),offset, ns+std::string("edge"));
      }
    }*/
}
/*void GraphPlot::PublishOctreeMap(perception_oru::graph_map::OctomapMapTypePtr octmap){

  octomap::OcTree *tree=octmap->GetTree();

}*/
void GraphPlot::PlotMap(NDTMap& map,
                        int color,
                        const Affine3d& offset,
                        PlotMarker marker,
                        std::string ns,
                        double point_size)
{

    if (marker == plotmarker::sphere)
    {
        // GraphPlot::SendGlobalMapToRviz(&map,color,offset);

        // toMessage(&map,)
    }
    else if (marker == plotmarker::point)
    {
        cov_vector cov;
        mean_vector mean;
        std::vector<double> occ;
        GetAllCellsMeanCov(&map, cov, mean, occ);
        // cout<<"mean size="<<mean.size()<<endl;
        PublishMapAsPoints(mean, color, point_size, offset, ns);
    }
}

void GraphPlot::plotParticleCloud(const Eigen::Affine3d& offset, std::vector<particle> pcloud)
{
    visualization_msgs::Marker marker;

    std_msgs::ColorRGBA pcolor;
    if (!initialized_)
        Initialize();

    marker.color.a = 0.7;
    marker.color.r = 0.8;
    marker.color.b = 0.1;
    marker.color.g = 0.1;
    marker.ns = "";
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    std_msgs::ColorRGBA c;
    c.a = 1.0;

    for (int i = 0; i < pcloud.size(); i++)
    {
        geometry_msgs::Point point;
        Eigen::Vector3d point_offset = offset * pcloud[i].GetAsAffine().translation();
        point.x = point_offset(0);
        point.y = point_offset(1);
        point.z = point_offset(2);
        marker.points.push_back(point);
    }
    particlaCloudPublisher_->publish(marker);
}

void GraphPlot::Initialize()
{
    cout << "graph_plot: initialize" << endl;
    nh_ = new ros::NodeHandle("");
    graphPosePublisher_ = new ros::Publisher();
    localMapPublisher_ = new ros::Publisher();
    globalMapPublisher_ = new ros::Publisher();
    global2MapPublisher_ = new ros::Publisher();
    graphInfoPublisher_ = new ros::Publisher();
    particlaCloudPublisher_ = new ros::Publisher();
    observationPublisher_ = new ros::Publisher();
    trajectoryPublisher_ = new ros::Publisher();
    map_pub_ = new ros::Publisher();

    *localMapPublisher_ = nh_->advertise<visualization_msgs::MarkerArray>(NDT_LOCAL_MAP_TOPIC, 10);
    *globalMapPublisher_ =
        nh_->advertise<visualization_msgs::MarkerArray>(NDT_GLOBAL_MAP_TOPIC, 10);
    *global2MapPublisher_ =
        nh_->advertise<visualization_msgs::MarkerArray>(NDT_GLOBAL2_MAP_TOPIC, 10);
    *graphPosePublisher_ = nh_->advertise<geometry_msgs::PoseArray>(GRAPH_POSE_TOPIC, 10);
    *graphInfoPublisher_ = nh_->advertise<visualization_msgs::MarkerArray>(GRAPH_INFO_TOPIC, 10);
    *particlaCloudPublisher_ = nh_->advertise<visualization_msgs::Marker>(PARTICLES_TOPIC, 10);
    *observationPublisher_ =
        nh_->advertise<visualization_msgs::MarkerArray>(OBSERVATIONS_TOPIC, 10);
    *trajectoryPublisher_ = nh_->advertise<geometry_msgs::PoseArray>(TRAJECTORY_TOPIC, 10);
    *map_pub_ = nh_->advertise<ndt_map_lamide::NDTMapRGBMsg>(NDT_OM_MAP_TOPIC, 10);
    initialized_ = true;
}

void GraphPlot::PlotNDT(pcl::PointCloud<pcl::PointXYZL>& cloud,
                        const std::string& frame_id,
                        const std::string& topic,
                        const Affine3d& offset)
{

    perception_oru::NDTMap* ndlocal = new NDTMap(new perception_oru::LazyGrid(0.8));
    ndlocal->guessSize(0, 0, 0, 130, 130, 20);
    ndlocal->loadPointCloud(cloud, 130);
    ndlocal->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
    PlotNDT(ndlocal, frame_id, topic, offset);
}

void GraphPlot::PlotScan(pcl::PointCloud<pcl::PointXYZL>& cloud,
                         const std::string& frame_id,
                         const std::string& topic,
                         const Affine3d& offset)
{
    ros::Publisher* pub;
    if (!FindPublisherHandle(topic, pub))
    {
        pub = new ros::Publisher();
        ros::NodeHandle nh("~");

        *pub = nh.advertise<pcl::PointCloud<pcl::PointXYZL>>(topic, 10);
        pub_handles.push_back(std::make_pair(topic, pub));
    }
    Eigen::Affine3d T = offset;
    transformPointCloudInPlace(T, cloud);
    cloud.header.frame_id = frame_id;
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud.width = cloud.size();
    cloud.height = 1;
    pub->publish(cloud);
}

void GraphPlot::PlotScan(pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>& cloud,
                         const std::string& frame_id,
                         const std::string& topic,
                         const Affine3d& offset)
{
    ros::Publisher* pub;
    if (!FindPublisherHandle(topic, pub))
    {
        pub = new ros::Publisher();
        ros::NodeHandle nh("~");

        *pub = nh.advertise<pcl::PointCloud<velodyne_pointcloud_oru::PointXYZIR>>(topic, 10);
        pub_handles.push_back(std::make_pair(topic, pub));
    }
    Eigen::Affine3d T = offset;
    transformPointCloudInPlace(T, cloud);
    cloud.header.frame_id = frame_id;
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud.width = cloud.size();
    cloud.height = 1;
    pub->publish(cloud);
}

void GraphPlot::PlotNDT(NDTMap*& ndt,
                        const std::string& frame_id,
                        const std::string& topic,
                        const Affine3d& offset)
{

    ros::Publisher* pub;
    if (!FindPublisherHandle(topic, pub))
    {
        ros::NodeHandle nh("~");
        pub = new ros::Publisher();
        *pub = nh.advertise<ndt_map_lamide::NDTMapRGBMsg>(topic, 10);
        pub_handles.push_back(std::make_pair(topic, pub));
    }
    if (ndt != NULL)
    {
        std::vector<NDTCell*> cells = ndt->pseudoTransformNDT(offset);
        ndt_map_lamide::NDTMapRGBMsg msg;
        toRGBMessage(cells, msg, frame_id);
        if (msg.cells.size() > 0)
            pub->publish(msg);
        for (NDTCell* p : cells)
        {
            delete p;
        }
    }
}

void GraphPlot::PlotNDTRGB(NDTMap*& ndt,
                           const std::string& frame_id,
                           const std::string& topic,
                           const Affine3d& offset,
                           int colors)
{

    ros::Publisher* pub;
    if (!FindPublisherHandle(topic, pub))
    {
        ros::NodeHandle nh("~");
        pub = new ros::Publisher();
        *pub = nh.advertise<ndt_map_lamide::NDTMapRGBMsg>(topic, 10);
        pub_handles.push_back(std::make_pair(topic, pub));
    }
    if (ndt != NULL)
    {
        std::vector<NDTCell*> cells = ndt->pseudoTransformNDT(offset);
        ndt_map_lamide::NDTMapRGBMsg msg;
        toRGBMessage(cells, msg, frame_id, colors);
        if (msg.cells.size() > 0)
            pub->publish(msg);
        for (NDTCell* p : cells)
        {
            delete p;
        }
    }
}

void GraphPlot::PlotNDT(std::vector<perception_oru::NDTCell*>& ndt,
                        const std::string& frame_id,
                        const std::string& topic)
{
    ros::Publisher* pub;
    if (!FindPublisherHandle(topic, pub))
    {
        ros::NodeHandle nh("~");
        pub = new ros::Publisher();
        *pub = nh.advertise<ndt_map_lamide::NDTMapRGBMsg>(topic, 10);
        pub_handles.push_back(std::make_pair(topic, pub));
    }
    if (!ndt.empty())
    {
        ndt_map_lamide::NDTMapRGBMsg msg;
        toRGBMessage(ndt, msg, frame_id);
        if (msg.cells.size() > 0)
            pub->publish(msg);
    }
}

bool GraphPlot::FindPublisherHandle(const std::string& topic, ros::Publisher*& pub)
{
    bool handle_found = false;
    for (int i = 0; i < pub_handles.size(); i++)
    {
        if (topic == pub_handles[i].first)
        {
            pub = pub_handles[i].second;
            handle_found = true;
        }
    }
    return handle_found;
}

/*void GraphPlot::PlotPoseGraph(GraphMapPtr graph){
  if(!initialized_)
    Initialize();
  //Publish poses
  geometry_msgs::PoseArray poseArr;
  poseArr.poses.clear();
  poseArr.header.stamp=ros::Time::now();
  poseArr.header.frame_id="/world";

  geometry_msgs::Pose pose;

  for(int i=0;i<graph->Size();i++){
    Affine3d pose_tmp= graph->GetNodePose(i);
    tf::poseEigenToMsg(pose_tmp,pose);
    poseArr.poses.push_back(pose);
  }

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker_array.markers.clear();
  marker.header.frame_id = "/world";
  marker.header.stamp = poseArr.header.stamp;
  marker.ns = "graphInfo";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime=ros::Duration();
  marker.scale.z=0.2;
  marker.color.a=1;
  marker.color.b=1;

  Affine3d pose_tmp= graph->GetCurrentNode()->GetMapPose(false);
  tf::poseEigenToMsg(pose_tmp,pose);
  marker.pose.position.x=pose_tmp.translation()(0);
  marker.pose.position.y=pose_tmp.translation()(1);
  marker.pose.position.z=pose_tmp.translation()(2)+1.0;

  std::ostringstream os;
  os << std::setw(3)<< "(x,y)=("<<marker.pose.position.x<<","<<marker.pose.position.y<<")"<<endl;
  marker.pose.orientation.w=1.0;
  marker.pose.orientation.z=0.0;
  marker.pose.orientation.y=0.0;
  marker.pose.orientation.x=0.0;


  marker.text= os.str();
  marker_array.markers.push_back(marker);
  marker.id=1;
  graphPosePublisher_->publish(poseArr);
  graphInfoPublisher_->publish(marker_array);


}*/

void GraphPlot::PlotObservationVector(GraphMapPtr graph_map)
{
    static unsigned max_id = 0;
    visualization_msgs::MarkerArray marker_arr;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = "observations";

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // marker.id = id;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.68;
    marker.color.g = 0.93;
    marker.color.b = 0.93;
    std_msgs::ColorRGBA p_color;
    p_color.a = 0.6;
    GraphPlot::ColorGradient color_map;
    color_map.CreateVisibleSpectrum();

    float r, g, b;

    unsigned int id = 0;
    ndt_generic::Affine3dSTLVek path_tmp;
    geometry_msgs::PoseArray pose_arr_msg;
    pose_arr_msg.header.stamp = ros::Time::now();
    pose_arr_msg.header.frame_id = "/world";

    if (GraphMapNavigatorPtr graph = boost::dynamic_pointer_cast<GraphMapNavigator>(graph_map))
    {
        for (mapNodeItr itr = graph->begin(); itr != graph->end(); itr++)
        {
            path_tmp = (*itr)->GetObservationVector();
            for (int i = 0; i < path_tmp.size(); i++)
            {
                geometry_msgs::Pose p_tmp;
                tf::poseEigenToMsg(path_tmp[i] * graph->GetSensorPose().inverse(), p_tmp);
                pose_arr_msg.poses.push_back(p_tmp);
            }
        }
    }
    if (GraphMapNavigatorPtr graph = boost::dynamic_pointer_cast<GraphMapNavigator>(graph_map))
    {
        for (int i = 0; i < graph_map->Size(); i++)
        { // step through graph and mark all color
            Eigen::Affine3d node = graph_map->GetNodePose(i);
            double color_index = (double)i / ((double)graph_map->Size());
            color_map.getColorAtValue(color_index, r, g, b);
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            node.translation()(2) = node.translation()(2) + 0.8;
            tf::poseEigenToMsg(node, marker.pose);
            marker.id = id++;
            marker_arr.markers.push_back(marker);

            if (max_id < id)
                max_id = id;
        }

        std::vector<MapNodePtr> closest_nodes =
            graph->GetClosestNodes(graph_map->GetCurrentNodePose(), DBL_MAX, true);
        for (int i = 0; i < closest_nodes.size(); i++)
        {
            ndt_generic::Affine3dSTLVek obs_vec = closest_nodes[i]->GetObservationVector();
            const unsigned int nodeid = closest_nodes[i]->GetId();
            double color_index = (double)nodeid / ((double)graph_map->Size());
            color_map.getColorAtValue(color_index, r, g, b);
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            for (int j = 0; j < obs_vec.size(); j++)
            {
                marker.id = id++;
                tf::poseEigenToMsg(obs_vec[j], marker.pose);

                if (max_id < id)
                    max_id = id;
                tf::poseEigenToMsg(obs_vec[j], marker.pose);
                marker.pose.position.z = marker.pose.position.z + ((double)color_index);
                marker_arr.markers.push_back(marker);
            }
        }
        for (int i = id; i < max_id; i++)
        { // remove already plotted unit which are not overwrited
            marker.action = visualization_msgs::Marker::DELETE;
            marker_arr.markers.push_back(marker);
        }
        trajectoryPublisher_->publish(pose_arr_msg);
        observationPublisher_->publish(marker_arr);
    }
}

void GraphPlot::makeRightHanded(Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues)
{
    // Note that sorting of eigenvalues may end up with left-hand coordinate system.
    // So here we correctly sort it so that it does end up being righ-handed and normalised.
    Eigen::Vector3d c0 = eigenvectors.block<3, 1>(0, 0);
    c0.normalize();
    Eigen::Vector3d c1 = eigenvectors.block<3, 1>(0, 1);
    c1.normalize();
    Eigen::Vector3d c2 = eigenvectors.block<3, 1>(0, 2);
    c2.normalize();
    Eigen::Vector3d cc = c0.cross(c1);
    if (cc.dot(c2) < 0)
    {
        eigenvectors << c1, c0, c2;
        double e = eigenvalues[0];
        eigenvalues[0] = eigenvalues[1];
        eigenvalues[1] = e;
    }
    else
    {
        eigenvectors << c0, c1, c2;
    }
}

void GraphPlot::computeShapeScaleAndOrientation3D(const Eigen::Matrix3d& covariance,
                                                  Eigen::Vector3d& scale,
                                                  Eigen::Quaterniond& orientation)
{
    Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
    Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());

    // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance
    // matrix
    // ORU_FIXME: Should we use Eigen's pseudoEigenvectors() ?
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
    // Compute eigenvectors and eigenvalues
    if (eigensolver.info() == Eigen::Success)
    {
        eigenvalues = eigensolver.eigenvalues();
        eigenvectors = eigensolver.eigenvectors();
    }
    else
    {
        ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the "
                             "covariance matrix correct?");
        eigenvalues =
            Eigen::Vector3d::Zero(); // Setting the scale to zero will hide it on the screen
        eigenvectors = Eigen::Matrix3d::Identity();
    }

    // Be sure we have a right-handed orientation system
    makeRightHanded(eigenvectors, eigenvalues);

    // Define the rotation

    orientation = Eigen::Quaterniond(eigenvectors);
    // Define the scale. eigenvalues are the variances, so we take the sqrt to draw the standard
    // deviation
    scale(0) = 2 * std::sqrt(eigenvalues[0]);
    scale(1) = 2 * std::sqrt(eigenvalues[1]);
    scale(2) = 2 * std::sqrt(eigenvalues[2]);
}

void GraphPlot::CovarToMarker(const Eigen::Matrix3d& cov,
                              const Eigen::Vector3d& mean,
                              visualization_msgs::Marker& marker)
{
    if (!initialized_)
        Initialize();

    Eigen::Vector3d scale_axis;
    Eigen::Quaterniond q_axis;
    // 3d plotting
    computeShapeScaleAndOrientation3D(cov, scale_axis, q_axis);
    marker.scale.x = scale_axis(0);
    marker.scale.y = scale_axis(1);
    marker.scale.z = scale_axis(2);
    if (marker.scale.x == 0)
        marker.scale.x == 1.0;
    if (marker.scale.y == 0)
        marker.scale.y == 1.0;
    if (marker.scale.z == 0)
        marker.scale.z == 1.0;
    marker.pose.orientation.x = q_axis.x();
    marker.pose.orientation.y = q_axis.y();
    marker.pose.orientation.z = q_axis.z();
    marker.pose.orientation.w = q_axis.w();

    /* --2d plotting
     Eigen::Matrix2d mat= cov.block(0,0,2,2);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(mat);
    Eigen::Vector2d eigValues = eig.eigenvalues();
    Eigen::Matrix2d eigVectors= eig.eigenvectors();

    double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));
    marker.scale.x=2*sqrt(eigValues(0));
    marker.scale.y=2*sqrt(eigValues(1));
    marker.scale.z=0.02;


    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = sin(angle*0.5);
    marker.pose.orientation.w = cos(angle*0.5); */

    marker.pose.position.x = mean(0);
    marker.pose.position.y = mean(1);
    marker.pose.position.z = mean(2);
}

void GraphPlot::PublishMapAsOccupancy(mean_vector& mean,
                                      const std::vector<double>& occupancy,
                                      double scale,
                                      const Eigen::Affine3d& offset,
                                      std::string ns,
                                      int id)
{

    if (!initialized_)
        Initialize();

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerarr;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.id = 0;

    mean_vector mean_vek;
    geometry_msgs::Point p;
    for (unsigned int i = 0; i < mean.size(); i++)
    {
        Eigen::Vector3d m_tmp = offset * mean[i];
        p.x = m_tmp(0);
        p.y = m_tmp(1);
        p.z = m_tmp(2);
        cerr << "MISSING ndt_visualisation::assignColor(marker, 3);" << endl;
        //  ndt_visualisation::assignColor(marker, 3);
        marker.color.a = 1.0 - (1.0 / (1 + exp(occupancy[i])));
        cout << "occ=" << occupancy[i] << endl;
        marker.pose.position = p;
        marker.id = i;
        markerarr.markers.push_back(marker);
    }

    globalMapPublisher_->publish(markerarr);
}

void GraphPlot::PublishMapAsPoints(mean_vector& mean,
                                   int color,
                                   double scale,
                                   const Eigen::Affine3d& offset,
                                   std::string ns,
                                   int id)
{

    if (!initialized_)
        Initialize();

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerarr;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.x = 0.0;

    geometry_msgs::Point p;
    float min_z, max_z;
    if (mean.size() > 0)
    {
        Eigen::Vector3d m_tmp = offset * mean[0];
        min_z = m_tmp(2);
        max_z = m_tmp(2);
    }
    mean_vector mean_vek;
    for (unsigned int i = 0; i < mean.size(); i++)
    {
        Eigen::Vector3d m_tmp = offset * mean[i];
        p.x = m_tmp(0);
        p.y = m_tmp(1);
        p.z = m_tmp(2);
        if (p.z < min_z)
            min_z = p.z;
        if (p.z > max_z)
            max_z = p.z;
        marker.points.push_back(p);
        mean_vek.push_back(m_tmp);
    }

    if (color < 0)
    {

        std_msgs::ColorRGBA p_color;
        p_color.a = 1.0;
        GraphPlot::ColorGradient color_map;
        color_map.createDefaultHeatMapGradient();
        for (int i = 0; i < mean_vek.size(); i++)
        {
            float r, g, b;
            float z = mean_vek[i](2);
            z = (z - min_z) / (max_z - min_z);
            color_map.getColorAtValue(z, r, g, b);
            p_color.r = r;
            p_color.g = g;
            p_color.b = b;
            marker.colors.push_back(p_color);
        }
    }
    else
    {
        cerr << "MISSING ndt_visualisation::assignColor(marker, 3);" << endl;
        // ndt_visualisation::assignColor(marker, color);
    }
    markerarr.markers.push_back(marker);
    globalMapPublisher_->publish(markerarr);
}

void GraphPlot::sendMapToRviz(mean_vector& mean,
                              cov_vector& cov,
                              ros::Publisher* mapPublisher,
                              string frame,
                              int color,
                              const Affine3d& offset,
                              string ns,
                              int markerType)
{
    static unsigned int max_size = 0;
    if (!initialized_)
        Initialize();

    if (max_size < mean.size())
        max_size = mean.size();

    visualization_msgs::MarkerArray marray;
    visualization_msgs::Marker marker;
    marray.markers.clear();
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.type = markerType;

    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    if (color == 0)
    {
        marker.color.a = 0.75;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else if (color == 1)
    {
        marker.color.a = 0.6;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else if (color == 2)
    {
        marker.color.a = 0.4;
        marker.color.r = 0.4;
        marker.color.g = 0.6;
        marker.color.b = 1.0;
    }
    for (unsigned int i = 0; i < mean.size(); i++)
    {
        Eigen::Matrix3d cov_tmp = offset.linear() * cov[i] * offset.linear().transpose();
        Eigen::Vector3d m_tmp = offset * mean[i];
        CovarToMarker(cov_tmp, m_tmp, marker);
        marker.id = i;
        marray.markers.push_back(marker);
    }
    marray.markers.push_back(marker);
    for (int i = mean.size(); i < max_size; i++)
    {
        marker.id = i;
        marray.markers.push_back(marker);
    }
    mapPublisher->publish(marray);
}

/*void GraphPlot::SendLocalMapToRviz(perception_oru::NDTMap *mapPtr,int color,const Affine3d
&offset){ cov_vector cov; mean_vector mean; std::vector<double> occ;

  if(!initialized_)
    Initialize();
  GetAllCellsMeanCov(mapPtr,cov,mean,occ);
  sendMapToRviz(mean,cov,localMapPublisher_,"/velodyne",color,offset,"local");
}*/
void GraphPlot::GetAllCellsMeanCov(const perception_oru::NDTMap* mapPtr,
                                   cov_vector& cov,
                                   mean_vector& mean,
                                   std::vector<double>& occ)
{
    cov.clear();
    mean.clear();
    std::vector<perception_oru::NDTCell*> cells = mapPtr->getAllCells();
    NDTCell* cell;
    for (int i = 0; i < cells.size(); i++)
    {
        cov.push_back(cells[i]->getCov());
        mean.push_back(cells[i]->getMean());
        occ.push_back(cells[i]->getOccupancy());
        // delete cells[i];
    }
}
void GraphPlot::GetAllCellsMeanCov(std::vector<perception_oru::NDTCell*> cells,
                                   cov_vector& cov,
                                   mean_vector& mean,
                                   std::vector<double>& occupancy)
{
    cov.clear();
    mean.clear();
    for (int i = 0; i < cells.size(); i++)
    {
        cov.push_back(cells[i]->getCov());
        mean.push_back(cells[i]->getMean());
        occupancy.push_back(cells[i]->getOccupancy());
    }
}
void GraphPlot::GetAllCellsOccupancy(std::vector<perception_oru::NDTCell*> cells,
                                     mean_vector& center_position,
                                     std::vector<double>& occupancy)
{
    center_position.clear();
    double x = 0, y = 0, z = 0;
    for (int i = 0; i < cells.size(); i++)
    {
        cells[i]->getCenter(x, y, z);
        center_position.push_back(Eigen::Vector3d(x, y, z));
        occupancy.push_back(cells[i]->getOccupancy());
    }
}

/*void GraphPlot::SendGlobalMapToRviz(perception_oru::NDTMap *mapPtr, int color,const Affine3d
&offset){ cov_vector cov; mean_vector mean; std::vector<double> occ; if(!initialized_) Initialize();

  GetAllCellsMeanCov(mapPtr,cov,mean,occ);
  sendMapToRviz(mean,cov,globalMapPublisher_,"world",color,offset,"current");
}
void GraphPlot::SendGlobal2MapToRviz(perception_oru::NDTMap *mapPtr, int color,const Affine3d
&offset){ cov_vector cov;//end namespacev; mean_vector mean; std::vector<double> occ;
  if(!initialized_)
    Initialize();
  GetAllCellsMeanCov(mapPtr,cov,mean,occ);
  sendMapToRviz(mean,cov,globalMapPublisher_,"world",color,offset,"prev",visualization_msgs::Marker::CUBE);
}
void GraphPlot::SendGlobal2MapToRviz(  std::vector<perception_oru::NDTCell*>cells, int color,const
Affine3d &offset){ cov_vector cov; mean_vector mean; std::vector<double> occ;

  if(!initialized_)
    Initialize();
  GetAllCellsMeanCov(cells,cov,mean,occ);
  sendMapToRviz(mean,cov,globalMapPublisher_,"world",color,offset,"prev",visualization_msgs::Marker::CUBE);
}*/

} // namespace graph_map
} // namespace perception_oru
