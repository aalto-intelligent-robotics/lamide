//#include <ndt_fuser.h>
#include <ndt_fuser_lamide/ndt_fuser_hmt.h>
#include <ndt_fuser_lamide/ndt_fuser_ros_wrappers/ndt_fuser_logger.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ndt_map_lamide/ndt_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/ndt_conversions.h>
#include <ndt_fuser_lamide/ndt_fuser_ros_wrappers/ros_fuser_init.hpp>

#include "ndt_feature_finder_lamide/ndt_corner.hpp"

#ifndef SYNC_FRAMES
#define SYNC_FRAMES 20
#define MAX_TRANSLATION_DELTA 2.0
#define MAX_ROTATION_DELTA 0.5
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Todor Stoyanov
 *
 */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> LaserPoseSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;

class NDTFuserNode {

protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
  ros::Subscriber gt_sub;

  // Components for publishing
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listener_;
  ros::Publisher output_pub_, laserpub;
  Eigen::Affine3d pose_, T, sensor_pose_;

  unsigned int nb_added_clouds_;
  double varz;

  boost::mutex m, message_m;
//   perception_oru::NDTFuserHMT *fuser_base;
  perception_oru::ndt_fuser::NDTFuserHMTLogger *fuser;
  std::string points_topic, laser_topic, map_dir, map_name, odometry_topic,
    world_frame, robot_frame, sensor_frame, fuser_frame, init_pose_frame, gt_topic, bag_name;
  double size_x, size_y, size_z, resolution, sensor_range, min_laser_range_;
  bool visualize, match2D, matchLaser, beHMT, useOdometry, plotGTTrack,
       initPoseFromGT, initPoseFromTF, initPoseSet, renderGTmap;

  double pose_init_x,pose_init_y,pose_init_z,
    pose_init_r,pose_init_p,pose_init_t;
  double sensor_pose_x,sensor_pose_y,sensor_pose_z,
    sensor_pose_r,sensor_pose_p,sensor_pose_t;
  laser_geometry::LaserProjection projector_;

  message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
  message_filters::Synchronizer< LaserPoseSync > *sync_lp_;

  message_filters::Synchronizer< PointsOdomSync > *sync_po_;
  message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
  ros::ServiceServer save_map_;

  ros::Publisher map_publisher_;

  Eigen::Affine3d last_odom, this_odom;
  std::string tf_pose_frame_;
  bool use_tf_listener_;
  Eigen::Affine3d last_tf_frame_;

  perception_oru::ndt_feature_finder::NDTCorner _ndt_corners;
	ros::Publisher _gaussian_pub;
	ros::Publisher _gaussian_pub2;
	ros::Publisher _corner_orientation_pub;
	ros::Publisher _lines_pub;
	ros::Publisher _ndt_corner_pub;
	ros::Publisher _ndt_map_pub;
	visualization_msgs::Marker _gaussian_that_gave_corners;
	visualization_msgs::Marker _gaussian_that_gave_corners2;
	visualization_msgs::Marker _corner_orientation;
	visualization_msgs::Marker _lines;
	visualization_msgs::Marker _ndt_corner_markers;

public:
  // Constructor
  NDTFuserNode(ros::NodeHandle param_nh) : nb_added_clouds_(0)
  {

	_ndt_corners.setNeighborSize(2);

	_gaussian_pub = param_nh.advertise<visualization_msgs::Marker>("gaussian_that_gave_corners", 10);
	_gaussian_pub2 = param_nh.advertise<visualization_msgs::Marker>("gaussian_that_gave_corners2", 10);
	_corner_orientation_pub = param_nh.advertise<visualization_msgs::Marker>("corner_orientation", 10);
	_ndt_corner_pub = param_nh.advertise<visualization_msgs::Marker>("corner_ndt_marker", 10);
	_ndt_map_pub = param_nh.advertise<ndt_map_lamide::NDTMapRGBMsg>("ndt_map_corner", 10);
	_lines_pub = param_nh.advertise<visualization_msgs::Marker>("lines", 10);

	_gaussian_that_gave_corners.type = visualization_msgs::Marker::LINE_LIST;
	_gaussian_that_gave_corners.header.frame_id = "/world";
	_gaussian_that_gave_corners.ns = "corner_extraction";
	_gaussian_that_gave_corners.id = 7;
	_gaussian_that_gave_corners.scale.x = 0.2;
	_gaussian_that_gave_corners.scale.y = 0.2;
	_gaussian_that_gave_corners.color.g = 1.0f;
	_gaussian_that_gave_corners.color.r = 0.5f;
	_gaussian_that_gave_corners.color.a = 1.0;

	_gaussian_that_gave_corners2.type = visualization_msgs::Marker::LINE_LIST;
	_gaussian_that_gave_corners2.header.frame_id = "/world";
	_gaussian_that_gave_corners2.ns = "corner_extraction";
	_gaussian_that_gave_corners2.id = 7;
	_gaussian_that_gave_corners2.scale.x = 0.2;
	_gaussian_that_gave_corners2.scale.y = 0.2;
	_gaussian_that_gave_corners2.color.r = 1.0f;
	_gaussian_that_gave_corners2.color.b = 1.0f;
	_gaussian_that_gave_corners2.color.a = 1.0;

	_corner_orientation.type = visualization_msgs::Marker::LINE_LIST;
	_corner_orientation.header.frame_id = "/world";
	_corner_orientation.ns = "corner_extraction";
	_corner_orientation.id = 7;
	_corner_orientation.scale.x = 0.2;
	_corner_orientation.scale.y = 0.2;
// 	_corner_orientation.color.b = 1.0f;
	_corner_orientation.color.b = 1.0f;
	_corner_orientation.color.a = 1.0;

	_ndt_corner_markers.type = visualization_msgs::Marker::POINTS;
	_ndt_corner_markers.header.frame_id = "/world";
	_ndt_corner_markers.ns = "corner_extraction";
	_ndt_corner_markers.id = 2;
	_ndt_corner_markers.scale.x = 0.5;
	_ndt_corner_markers.scale.y = 0.5;
	_ndt_corner_markers.color.g = 1.0f;
	_ndt_corner_markers.color.a = 1.0;

	_lines.type = visualization_msgs::Marker::LINE_LIST;
	_lines.header.frame_id = "/world";
	_lines.ns = "corner_extraction";
	_lines.id = 3;
	_lines.scale.x = 0.1;
	_lines.scale.y = 0.1;
	_lines.color.b = 1.0f;
	_lines.color.a = 1.0;

	ROS_INFO( "INIT ON" );


	laserpub = param_nh.advertise<sensor_msgs::PointCloud2>("laser_read_logger", 10);

    ///if we want to build map reading scans directly from bagfile
    param_nh.param<std::string>("bagfile_name",bag_name,"data.bag");

    ///topic to wait for point clouds, if available
    param_nh.param<std::string>("points_topic",points_topic,"points");
    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");

	std::cout << "Subscribed to " << laser_topic << std::endl;

    ///if using the HMT fuser, NDT maps are saved in this directory.
    ///a word of warning: if you run multiple times with the same directory,
    ///the old maps are loaded automatically
    param_nh.param<std::string>("map_directory",map_dir,"map");
    param_nh.param<std::string>("map_name_prefix",map_name,"");

    ///initial pose of the vehicle with respect to the map
    param_nh.param("pose_init_x",pose_init_x,0.);
    param_nh.param("pose_init_y",pose_init_y,0.);
    param_nh.param("pose_init_z",pose_init_z,0.);
    param_nh.param("pose_init_r",pose_init_r,0.);
    param_nh.param("pose_init_p",pose_init_p,0.);
    param_nh.param("pose_init_t",pose_init_t,0.);

    ///pose of the sensor with respect to the vehicle odometry frame
    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
    param_nh.param("sensor_pose_t",sensor_pose_t,0.);

    ///size of the map in x/y/z. if using HMT, this is the size of the central tile
    param_nh.param("size_x_meters",size_x,10.);
    param_nh.param("size_y_meters",size_y,10.);
    param_nh.param("size_z_meters",size_z,10.);

    ///range to cutoff sensor measurements
    param_nh.param("sensor_range",sensor_range,3.);
    ///range to cutoff sensor measurements
    param_nh.param("min_laser_range",min_laser_range_,0.1);

    //map resolution
    param_nh.param("resolution",resolution,0.10);
    param_nh.param("laser_variance_z",varz,resolution/4);

    ///visualize in a local window
    param_nh.param("visualize",visualize,true);
    ///only mathc with 3dof
    param_nh.param("match2D",match2D,false);
    ///use HMT grid or simple grid.
    param_nh.param("beHMT",beHMT,false);
    ///use standard odometry messages for initial guess
    param_nh.param("useOdometry",useOdometry,false);
    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");
    ///if we want to compare to a ground truth topic
    param_nh.param("plotGTTrack",plotGTTrack,false);
    param_nh.param<std::string>("gt_topic",gt_topic,"groundtruth");
    ///if we want to get the initial pose of the vehicle relative to a different frame
    param_nh.param("initPoseFromGT",initPoseFromGT,false);
    //plot the map from the GT track if available
    param_nh.param("renderGTmap", renderGTmap,false);
    renderGTmap &= plotGTTrack; //can't render if we don't have it
    //get it from TF?
    param_nh.param("initPoseFromTF",initPoseFromTF,false);
    //the frame to initialize to
    param_nh.param<std::string>("init_pose_frame",init_pose_frame,"/state_base_link");
	//The robot frame to initialize the fuser to
    param_nh.param<std::string>("robot_frame",robot_frame,"/base_link");
	//The sensor frame to initialize the fuser to
    param_nh.param<std::string>("sensor_frame",sensor_frame,"/laser_frame");
    //the world frame
    param_nh.param<std::string>("world_frame",world_frame,"/world");
    //our frame
    param_nh.param<std::string>("fuser_frame",fuser_frame,"/fuser");

    ///enable for LaserScan message input
    param_nh.param("matchLaser",matchLaser,false);

    param_nh.param<std::string>("tf_pose_frame", tf_pose_frame_, std::string(""));

    perception_oru::MotionModel2d::Params motion_params;
    param_nh.param<double>("motion_params_Cd", motion_params.Cd, 0.005);
    param_nh.param<double>("motion_params_Ct", motion_params.Ct, 0.01);
    param_nh.param<double>("motion_params_Dd", motion_params.Dd, 0.001);
    param_nh.param<double>("motion_params_Dt", motion_params.Dt, 0.01);
    param_nh.param<double>("motion_params_Td", motion_params.Td, 0.001);
    param_nh.param<double>("motion_params_Tt", motion_params.Tt, 0.005);

    bool do_soft_constraints;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);

    use_tf_listener_ = false;
    if (tf_pose_frame_ != std::string("")) {
      use_tf_listener_ = true;
    }

    pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
      Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;

    sensor_pose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
      Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

    map_publisher_=nh_.advertise<ndt_map_lamide::NDTMapRGBMsg>("ndt_map",1000);

    if(matchLaser) match2D=true;

	std::cout << "Resolution " << resolution << std::endl;

	fuser = new perception_oru::ndt_fuser::NDTFuserHMTLogger("/home/malcolm/Documents/log_fuser/log_fuser.txt", resolution,size_x,size_y,size_z, sensor_range, visualize,match2D, false, false, 30, map_name, beHMT, map_dir, true, do_soft_constraints);

	;fuser->setMotionParams(motion_params);
    fuser->setSensorPose(sensor_pose_);

    if(!matchLaser) {
      points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,1);
      if(useOdometry) {
        odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
        sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
        sync_po_->registerCallback(boost::bind(&NDTFuserNode::points2OdomCallback, this, _1, _2));
      }
      else {
        points2_sub_->registerCallback(boost::bind( &NDTFuserNode::points2Callback, this, _1));
      }
    }
    else
    {


	std::cout << "Laser callback sub sub" << std::endl;
	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,laser_topic,2);
	if(useOdometry && !renderGTmap) {
		std::cout << "which" << std::endl;
	    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
	    sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
	    sync_lo_->registerCallback(boost::bind(&NDTFuserNode::laserOdomCallback, this, _1, _2));
	}
	else if(!renderGTmap){
		std::cout << "which1" << std::endl;
	    laser_sub_->registerCallback(boost::bind( &NDTFuserNode::laserCallback, this, _1));
	} else {
		std::cout << "which2" << std::endl;
	    //this render map directly from GT
	    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,gt_topic,10);
	    sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
	    sync_lo_->registerCallback(boost::bind(&NDTFuserNode::laserOdomCallback, this, _1, _2));
	    fuser->disableRegistration = true;
	}
    }
    save_map_ = param_nh.advertiseService("save_map", &NDTFuserNode::save_map_callback, this);

    if(plotGTTrack) {
	gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&NDTFuserNode::gt_callback, this);
    }

    initPoseSet = false;
  }

  ~NDTFuserNode()
  {
    delete fuser;
  }

  void processFrame(pcl::PointCloud<pcl::PointXYZL> &cloud,
                    Eigen::Affine3d Tmotion) {

    m.lock();
    if (nb_added_clouds_  == 0)
      {
		ROS_INFO("initializing fuser map. Init pose from GT? %d, TF? %d", initPoseFromGT, initPoseFromTF);
		if(initPoseFromGT) {
          //check if initial pose was set already
          if(!initPoseSet) {
			ROS_WARN("skipping frame, init pose not acquired yet!");
			m.unlock();
			return;
          }
          ROS_INFO("Init pose is (%lf,%lf,%lf)", pose_.translation()(0), pose_.translation()(1),
                 pose_.rotation().eulerAngles(0,1,2)(0));
			fuser->initialize(pose_,cloud);
			std::cout << "Saving map of cell : " << fuser->map->getAllCells().size() << std::endl;
// 	  		exit(0);
		}
		else if(initPoseFromTF){
			std::cout <<"will init " << robot_frame << " " << sensor_frame << std::endl;

			perception_oru::ndt_fuser::initSensorPose(*fuser, robot_frame, sensor_frame);
			std::cout <<"will init " << world_frame << " " << robot_frame << std::endl;
			perception_oru::ndt_fuser::initRobotPose(*fuser, cloud, world_frame, robot_frame);
			ROS_INFO(" all init");
// 			fuser->setSensorPose(robot_frame, sensor_frame);
// 			fuser->initialize(cloud, world_frame, robot_frame);
			ROS_INFO("OUT");
			std::cout << "Saving map of cell : " << fuser->map->getAllCells().size() << " with cloud " << cloud.size() << std::endl;

			fuser->print();
// 	  		exit(0);
		}
		nb_added_clouds_++;
      } else {
      //sanity check for odometry
      if((Tmotion.translation().norm() <0.01 && Tmotion.rotation().eulerAngles(0,1,2)(2)< 0.01) && useOdometry) {
        std::cerr<<"No motion, skipping Frame\n";
	m.unlock();
	return;
      }
      if(Tmotion.translation().norm() > MAX_TRANSLATION_DELTA) {
        std::cerr<<"Ignoring Odometry (max transl)!\n";
        std::cerr<<Tmotion.translation().transpose()<<std::endl;
        Tmotion.setIdentity();
      }
      if(Tmotion.rotation().eulerAngles(0,1,2)(2) > MAX_ROTATION_DELTA) {
        std::cerr<<"Ignoring Odometry (max rot)!\n";
        std::cerr<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
        Tmotion.setIdentity();
      }
      nb_added_clouds_++;
      pose_ = fuser->update(Tmotion,cloud);




    }
    m.unlock();
    tf::Transform transform;
#if ROS_VERSION_MINIMUM(1,9,0)
    //groovy
    tf::transformEigenToTF(pose_, transform);
#else
    //fuerte
    tf::TransformEigenToTF(pose_, transform);
#endif
    tf_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, fuser_frame));



	//Extract Corners

	extractCorners();

	std::cout << "Exporting the map of size " << fuser->map->getAllInitializedCellsShared().size() << std::endl;
	ndt_map_lamide::NDTMapRGBMsg mapmsg;
	mapmsg.header.stamp = ros::Time::now();
	perception_oru::toRGBMessage(fuser->map, mapmsg, "/world");
	_ndt_map_pub.publish(mapmsg);



  }

  void extractCorners(){

	  auto corners = _ndt_corners.getAllCorners(*(fuser->map));
	  std::cout << "Found " << corners.size() << " corners " << std::endl;
	  drawCorners(corners);
  }

	void drawCorners(const std::vector <perception_oru::ndt_feature_finder::NDTCornerBundle >& corners)
	{
			std::cout << "Drawing the gaussians " << std::endl;
	// 		int a ;
	// 		std::cin >> a;

		_lines.header.stamp = ros::Time::now();
		_lines.points.clear();

		_gaussian_that_gave_corners.points.clear();
		_gaussian_that_gave_corners2.points.clear();

		_gaussian_that_gave_corners.header.stamp = ros::Time::now();
		_gaussian_that_gave_corners2.header.stamp = ros::Time::now();

		_ndt_corner_markers.points.clear();
		_ndt_corner_markers.header.stamp = ros::Time::now();

		_corner_orientation.points.clear();
		_corner_orientation.header.stamp = ros::Time::now();

	// 		std::cout << "Getting the angles" << landmark.size() << std::endl;
	// 		std::cout << "Getting the corners " << edges.size() << std::endl;

		for(auto it = corners.begin() ; it != corners.end() ; ++it){

			assert((*it).getCells1().size() == (*it).getCells2().size());

			for(auto it_ndt = (*it).getCells1().begin() ; it_ndt != (*it).getCells1().end() ; ++it_ndt){

				Eigen::Vector3d mean = (*it_ndt)->getMean();
				auto angle = perception_oru::ndt_feature_finder::NDTCellAngle(**it_ndt);

				geometry_msgs::Point p2;
				p2.x = mean(0) + (0.5 * std::cos(angle) );
				p2.y = mean(1) + (0.5 * std::sin(angle) );
				p2.z = 0;
				geometry_msgs::Point p3;
				p3.x = mean(0) - (0.5 * std::cos(angle) );
				p3.y = mean(1) - (0.5 * std::sin(angle) );
				p3.z = 0;

				_gaussian_that_gave_corners.points.push_back(p2);
				_gaussian_that_gave_corners.points.push_back(p3);
				geometry_msgs::Point p4;
				p4.x = mean(0);
				p4.y = mean(1);
				p4.z = 0;
				_lines.points.push_back(p4);
				p4.x = it->getMean()(0);
				p4.y = it->getMean()(1);
				p4.z = 0;
				_lines.points.push_back(p4);


			}

			for(auto it_ndt = (*it).getCells2().begin() ; it_ndt != (*it).getCells2().end() ; ++it_ndt){

				Eigen::Vector3d mean = (*it_ndt)->getMean();
				auto angle = perception_oru::ndt_feature_finder::NDTCellAngle(**it_ndt);

				geometry_msgs::Point p2;
				p2.x = mean(0) + (0.5 * std::cos(angle) );
				p2.y =  mean(1) + (0.5 * std::sin(angle) );
				p2.z = 0;
				geometry_msgs::Point p3;
				p3.x = mean(0) - (0.5 * std::cos(angle) );
				p3.y = mean(1) - (0.5 * std::sin(angle) );
				p3.z = 0;

				_gaussian_that_gave_corners2.points.push_back(p2);
				_gaussian_that_gave_corners2.points.push_back(p3);

				geometry_msgs::Point p4;
				p4.x = mean(0);
				p4.y = mean(1);
				p4.z = 0;
				_lines.points.push_back(p4);
				p4.x = it->getMean()(0);
				p4.y = it->getMean()(1);
				p4.z = 0;
				_lines.points.push_back(p4);


			}
			geometry_msgs::Point p;
			p.x = it->getMean()(0);
			p.y = it->getMean()(1);
			p.z = 0;
			_ndt_corner_markers.points.push_back(p);

			drawOrientation(*it);

		}

	// 		std::cout << "Size " << _gaussian_that_gave_corners.points.size() << std::endl;

		_gaussian_pub.publish(_gaussian_that_gave_corners);
		_gaussian_pub2.publish(_gaussian_that_gave_corners2);

		_corner_orientation_pub.publish(_corner_orientation);



		std::cout << "Getting the corners" << std::endl;
		_ndt_corner_pub.publish(_ndt_corner_markers);

		_lines_pub.publish(_lines);



	}


	void drawOrientation(const perception_oru::ndt_feature_finder::NDTCornerBundle& corner_bundle)
	{
// 		std::cout << "Getting the angles" << std::endl;


		// 		std::cout << "Getting the angles" << landmark.size() << std::endl;
// 		std::cout << "Getting the corners " << edges.size() << std::endl;

		geometry_msgs::Point p;
// 				VertexLandmarkNDT* ptr = dynamic_cast<g2o::VertexPointXY*>((*it));
		auto vertex = corner_bundle.getMean();
		//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
		p.x = vertex(0);
		p.y = vertex(1);
		p.z = 0;


		double x, y, z;
		fuser->map->getCellSizeInMeters(x, y, z);

// 				std::cout << "getting the angle" << std::endl;
		int i = 0;
		for(auto it = corner_bundle.getOrientations().begin() ; it != corner_bundle.getOrientations().end(); ++it){

			assert(corner_bundle.getOrientations().size() == corner_bundle.getAngles().size());
			_corner_orientation.points.push_back(p);
			double angle = *it;

			std::cout << "angle " << angle<< std::endl;
			geometry_msgs::Point p2;
			p2.x = p.x + (2 * x * std::cos(angle) );
			p2.y = p.y + (2 * x * std::sin(angle) );
			p2.z = 0;
			_corner_orientation.points.push_back(p2);

			double anglew = corner_bundle.getAngles()[i];

			std::cout << "angle " << angle<< std::endl;
			p2.x = p.x + (2 * std::cos(angle - (anglew/2)) );
			p2.y = p.y + (2 * std::sin(angle - (anglew/2)) );
			p2.z = 0;
			_corner_orientation.points.push_back(p);
			_corner_orientation.points.push_back(p2);

// 			std::cout << "angle " << angle<< std::endl;
			p2.x = p.x + (2 * std::cos(angle + (anglew/2)) );
			p2.y = p.y + (2 * std::sin(angle + (anglew/2)) );
			p2.z = 0;
			_corner_orientation.points.push_back(p);
			_corner_orientation.points.push_back(p2);
			++i;
		}


//
// 				std::cout << "Line " << p << " "<< p2 << std::endl;;

	}





  bool save_map_callback(std_srvs::Empty::Request  &req,
                         std_srvs::Empty::Response &res ) {

    bool ret = false;
    ROS_INFO("Saving current map to map directory %s", map_dir.c_str());
    m.lock();
    ret = fuser->saveMap();
    m.unlock();
    return ret;
  }

  inline bool getAffine3dTransformFromTF(const ros::Time &time, Eigen::Affine3d& ret) {
    tf::StampedTransform transform;
    tf_listener_.waitForTransform("/world", tf_pose_frame_, time,ros::Duration(1.0));
    try{
      tf_listener_.lookupTransform("/world", tf_pose_frame_, time, transform);
      tf::poseTFToEigen(transform, ret);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
    return true;
  }

  // Callback
  void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
  {

    ROS_INFO_STREAM("points2Callback");
    //ROS_INFO_STREAM("last_odom : " << last_odom);
    pcl::PointCloud<pcl::PointXYZL> cloud;
    message_m.lock();
    pcl::fromROSMsg (*msg_in, cloud);
    message_m.unlock();
    T.setIdentity();
    if (!use_tf_listener_) {
      this->processFrame(cloud,T);
      publish_map();
      return;
    }

    // TF...

    Eigen::Affine3d transf, Tm;
    if (!getAffine3dTransformFromTF(msg_in->header.stamp, transf)) {
      ROS_ERROR("Failed to find the transform, will ignore these points");
      return;
    }
    ROS_INFO_STREAM("transf: "<<transf.translation().transpose()<<" "<<transf.rotation().eulerAngles(0,1,2));
    if (nb_added_clouds_  == 0)
    {
      last_tf_frame_ = transf;
      Tm.setIdentity();
    }
    else {
      Tm = last_tf_frame_.inverse()*transf;
      ROS_INFO_STREAM("delta from last update: "<<Tm.translation().transpose()<<" "<<Tm.rotation().eulerAngles(0,1,2)[2]);
      last_tf_frame_ = transf;
    }
    this->processFrame(cloud,Tm);
    /////////////////////////MAP PUBLISHIGN///////////////////////////
    publish_map();
  }

  // Callback
  void points2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                           const nav_msgs::Odometry::ConstPtr& odo_in)
  {

    ROS_INFO("got points2OdomCallback()");

    Eigen::Quaterniond qd;
    Eigen::Affine3d Tm;
    pcl::PointCloud<pcl::PointXYZL> cloud;

    message_m.lock();
    qd.x() = odo_in->pose.pose.orientation.x;
    qd.y() = odo_in->pose.pose.orientation.y;
    qd.z() = odo_in->pose.pose.orientation.z;
    qd.w() = odo_in->pose.pose.orientation.w;

    this_odom = Eigen::Translation3d (odo_in->pose.pose.position.x,
                                      odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;
    if (nb_added_clouds_  == 0)
      {
		Tm.setIdentity();
      } else {
      Tm = last_odom.inverse()*this_odom;
      std::cout<<"delta from last update: "<<Tm.translation().transpose()<<" "<<Tm.rotation().eulerAngles(0,1,2)[2] << std::endl;
      //if(Tm.translation().norm()<0.2 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(5*M_PI/180.0)) {
      //    message_m.unlock();
      //    return;
      //}
    }
    last_odom = this_odom;

    pcl::fromROSMsg (*msg_in, cloud);
    message_m.unlock();

    this->processFrame(cloud,Tm);
    /////////////////////////MAP PUBLISHIGN///////////////////////////
    publish_map();
    ROS_INFO("got points2OdomCallback() - done.");

  };

  // Callback
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
	  std::cout << "Laser callback" << std::endl;
    // Add to a queue
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZL> pcl_cloud_unfiltered, pcl_cloud;
    message_m.lock();
    projector_.projectLaser(*msg_in, cloud);
    message_m.unlock();
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZL pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    ROS_INFO("Got laser points");
    T.setIdentity();
    this->processFrame(pcl_cloud,T);
    /////////////////////////MAP PUBLISHIGN///////////////////////////
    publish_map();
  };

  // Callback
  void laserOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
                         const nav_msgs::Odometry::ConstPtr& odo_in)
  {
    ROS_INFO("Got laser odom points");
    Eigen::Quaterniond qd;
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZL> pcl_cloud, pcl_cloud_unfiltered;
    Eigen::Affine3d Tm;

    message_m.lock();
    qd.x() = odo_in->pose.pose.orientation.x;
    qd.y() = odo_in->pose.pose.orientation.y;
    qd.z() = odo_in->pose.pose.orientation.z;
    qd.w() = odo_in->pose.pose.orientation.w;

    this_odom = Eigen::Translation3d (odo_in->pose.pose.position.x,
                                      odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;

    //std::cout<<"AT: "<<this_odom.translation().transpose()<<" "<<this_odom.rotation().eulerAngles(0,1,2)[2] << std::endl;

    if (nb_added_clouds_  == 0)
      {
		Tm.setIdentity();
      } else {
      Tm = last_odom.inverse()*this_odom;
      //std::cout<<"delta from last update: "<<Tm.translation().transpose()<<" "<<Tm.rotation().eulerAngles(0,1,2)[2] << std::endl;
      //if(Tm.translation().norm()<0.2 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(5*M_PI/180.0)) {
      //    message_m.unlock();
      //    return;
      //}
    }
    last_odom = this_odom;

    projector_.projectLaser(*msg_in, cloud);
    message_m.unlock();

    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZL pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }

	sensor_msgs::PointCloud2 mesg;
	mesg.header.stamp = ros::Time::now();
	pcl::toROSMsg (pcl_cloud, mesg);
	mesg.header.frame_id = "/velodyne";
	mesg.header.stamp = ros::Time::now();
	laserpub.publish<sensor_msgs::PointCloud2>(mesg);

    //ROS_INFO("Got laser and odometry!");
    this->processFrame(pcl_cloud,Tm);
    /////////////////////////MAP PUBLISHIGN///////////////////////////
    publish_map();
    /////////////////////////////////////////////////
  };

  // Callback
  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
  {
    Eigen::Quaterniond qd;
    Eigen::Affine3d gt_pose;

    qd.x() = msg_in->pose.pose.orientation.x;
    qd.y() = msg_in->pose.pose.orientation.y;
    qd.z() = msg_in->pose.pose.orientation.z;
    qd.w() = msg_in->pose.pose.orientation.w;

    gt_pose = Eigen::Translation3d (msg_in->pose.pose.position.x,
                                    msg_in->pose.pose.position.y,msg_in->pose.pose.position.z) * qd;

    //ROS_INFO("got GT pose from GT track");
    m.lock();
    if(initPoseFromGT && !initPoseSet) {
      initPoseSet = true;
      pose_ = gt_pose;
      ROS_INFO("Set initial pose from GT track");
    }
    if(visualize){
#ifndef NO_NDT_VIZ
      fuser->viewer->addTrajectoryPoint(gt_pose.translation()(0),gt_pose.translation()(1),gt_pose.translation()(2)+0.2,1,1,1);
      fuser->viewer->displayTrajectory();
      //      fuser->viewer->repaint();
#endif
    }
    m.unlock();
  }

public:
  // map publishing function
  bool publish_map(){
// #if 0
    ndt_map_lamide::NDTMapRGBMsg map_msg;
    perception_oru::toRGBMessage(fuser->map, map_msg,world_frame);
    map_publisher_.publish(map_msg);
// #endif
return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "corner_extraction");

  ros::NodeHandle param("~");
  NDTFuserNode t(param);
  while(ros::ok()){
	  ros::spinOnce();
  }

  return 0;
}

