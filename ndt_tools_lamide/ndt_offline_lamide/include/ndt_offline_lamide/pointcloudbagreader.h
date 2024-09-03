#ifndef POINTCLOUDBAGREADER_H
#define POINTCLOUDBAGREADER_H

// Workaround for lz4 conflicting declaration -----------------------------//
// see https://github.com/ethz-asl/lidar_align/issues/16#issuecomment-758960226
#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
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

#include <ros/ros.h>
//#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud_oru/rawdata.h>
#include <ndt_generic_lamide/point_types.h>
#include <ndt_offline_lamide/PoseInterpolationNavMsgsOdo.h>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include <eigen_conversions/eigen_msg.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "ndt_generic_lamide/pcl_utils.h"
typedef pcl::PointCloud<pcl::PointXYZL> PointCloud;
template<typename PointT>
class PointCloudBagReader{
public:
  /**
   * Constructor
   * @param calibration_file path and name to your velodyne calibration file
   * @param bagfilename The path and name of the bagfile you want to handle
   * @param velodynetopic The topic that contains velodyne_msgs/VelodyneScan
   * @param tf_pose_id The id of the tf that you want to use
   * @param fixed_frame_id The name of the fixed frame in tf (default = "/world")
   * @param tftopic name of tf (default "/tf")
   * @param dur The buffer size (must be larger than the length of the bag file gives) default = 3600s
   * @param sensor_link An optional static link that takes e.g. your /odom to the sensor frame
   */
  PointCloudBagReader(std::string calibration_file,
                      std::string bagfilename,
                      const std::string & point_cloud_topic,
                      std::string tf_pose_id,
                      std::string fixed_frame_id="/world",
                      std::string tftopic="/tf",
                      ros::Duration dur = ros::Duration(3600),
                      tf::StampedTransform *sensor_link=NULL,
                      double velodyne_max_range=130.0,
                      double velodyne_min_range=2.0,
                      double sensor_time_offset=0.0
      )

  {
    // The view_direction / view_width is important to have set. The min/max range is overwritten in the setupOffline

    double view_direction = 0;
    double view_width = 2*M_PI;
    max_range=velodyne_max_range;
    min_range=velodyne_min_range;
    dataParser.setParameters(velodyne_min_range,
                             velodyne_max_range,
                             view_direction,
                             view_width);

    dataParser.setupOffline(calibration_file, velodyne_max_range, velodyne_min_range);
    sensor_time_offset_ = ros::Duration(sensor_time_offset);

    std::cout<<"Opening bagfile: "<<bagfilename.c_str()<<std::endl;
    bag.open(bagfilename, rosbag::bagmode::Read);

    point_cloud_topic_ = point_cloud_topic;
    tf_pose_id_ = tf_pose_id;

    std::vector<std::string> topics;
    topics.push_back(tftopic);
    topics.push_back(point_cloud_topic);


    for(int i=0; i<topics.size(); ++i) {
      std::cout<<"Searched Topic ["<<i<<"]="<<topics[i].c_str()<<std::endl;
    }

    view = new rosbag::View(bag, rosbag::TopicQuery(topics));
    I = view->begin();
    odosync = new PoseInterpolationNavMsgsOdo(view,tftopic, fixed_frame_id,dur, sensor_link);

    //odosync = NULL;
  }

  /**
   * Reads the next measurment.
   * @param cloud The generated point cloud
   * @param sensor_pose [out] The pose of the sensor origin. (Utilizes the tf_pose_id and sensor_link from the constructor).
   **/

  void sensorMessageToPclCloud(const sensor_msgs::PointCloudConstPtr &source, pcl::PointCloud<pcl::PointXYZL> &target){

    target.header.frame_id="/velodyne";
    target.header.seq=source->header.seq;
    pcl_conversions::toPCL(source->header.stamp,target.header.stamp);

    for(int i=0;i<source->points.size();i++){
      pcl::PointXYZL p;
      p.x=source->points[i].x;
      p.y=source->points[i].y;
      p.z=source->points[i].z;
      target.push_back(p);
    }
  }
void FilterRange(const pcl::PointCloud<PointT> &cloud_input, pcl::PointCloud<PointT> &cloud_filtered){
  cloud_filtered.clear();

  for (int i = 0; i < cloud_input.size(); ++i) {
    Eigen::Vector3d vek;
    vek<<cloud_input.points[i].x,cloud_input.points[i].y,cloud_input.points[i].z;
    if( min_range < vek.norm() && vek.norm()<max_range)
      cloud_filtered.push_back(cloud_input.points[i]);
  }
  cloud_filtered.width=cloud_filtered.size();
  cloud_filtered.height=1;
  cloud_filtered.header.stamp=cloud_input.header.stamp;
  cloud_filtered.header.frame_id=cloud_input.header.frame_id;
}

  bool readNextMeasurement(pcl::PointCloud<PointT> &cloud){
    pcl::PointCloud<PointT> cloud_unfiltered;
    cloud.clear();
    bool found_scan=false;
    while(!found_scan){
      if(I == view->end()){
        std::cout<<"End of measurment file reached"<<std::endl;
        return false;
      }
      rosbag::MessageInstance const m = *I;
      if(m.getTopic()==point_cloud_topic_){
        std::cout<<"found scan with topic :"<<point_cloud_topic_<<std::endl;
        sensor_msgs::LaserScanConstPtr laser_scan = m.instantiate<sensor_msgs::LaserScan>();
        sensor_msgs::PointCloudConstPtr point_cloud = m.instantiate<sensor_msgs::PointCloud>();
        sensor_msgs::PointCloud2ConstPtr point_cloud2 = m.instantiate<sensor_msgs::PointCloud2>();


        if ( point_cloud != NULL){
          found_scan=true;
          for(int i=0;i<point_cloud->points.size();i++){
            PointT p_target;
            geometry_msgs::Point32 p_src = point_cloud->points[i];
            p_target.x=p_src.x;
            p_target.y=p_src.y;
            p_target.z=p_src.z;
            cloud_unfiltered.push_back(p_target);
          }
          cloud_unfiltered.header.frame_id=point_cloud->header.frame_id;
          pcl_conversions::toPCL(point_cloud->header.stamp,cloud_unfiltered.header.stamp);
          timestamp_of_last_sensor_message=point_cloud->header.stamp;
        }
        else if(point_cloud2 != NULL){
          found_scan=true;
          pcl::PCLPointCloud2 pcl_pc2;
          pcl_conversions::toPCL(*point_cloud2,pcl_pc2);
          pcl::fromPCLPointCloud2(pcl_pc2,cloud_unfiltered);
          timestamp_of_last_sensor_message=point_cloud2->header.stamp;
        }
        else if(laser_scan!= NULL){
          found_scan=true;
          sensor_msgs::PointCloud2 sens_cloud;
          projector_.projectLaser(*laser_scan, sens_cloud);
          pcl::PCLPointCloud2 pcl_pc2;
          pcl_conversions::toPCL(sens_cloud,pcl_pc2);
          pcl::fromPCLPointCloud2(pcl_pc2,cloud_unfiltered);
          pcl_conversions::toPCL( laser_scan->header.stamp,cloud.header.stamp);
          timestamp_of_last_sensor_message=laser_scan->header.stamp;
          ndt_generic::AddVariance(cloud_unfiltered,0.05);
        }
      }
      I++;
    }
    FilterRange(cloud_unfiltered,cloud);
    return true;
  }
  void CloseOutputBag(){
    outbag.close();
  }

  /**
       * Get pose for latest measurement with pose id
       */
  bool getPoseFor(tf::Transform &pose, std::string pose_id){
    if(odosync->getTransformationForTime(timestamp_of_last_sensor_message,pose_id,pose)){
      return true;
    }
    return false;
  }


  ros::Time getTimeStampOfLastSensorMsg() const {
    return timestamp_of_last_sensor_message;
  }

private:
  unsigned int counter=0;
  rosbag::Bag outbag;
  std::string outbag_name_;
  PoseInterpolationNavMsgsOdo *odosync;
  velodyne_rawdata_oru::RawData dataParser;
  laser_geometry::LaserProjection projector_;
  rosbag::Bag bag;
  rosbag::View *view;
  rosbag::View::iterator I;
  std::string point_cloud_topic_;
  std::string tf_pose_id_;
  velodyne_msgs::VelodyneScan::ConstPtr global_scan;
  ros::Time timestamp_of_last_sensor_message;
  ros::Duration sensor_time_offset_;
  double min_range=0,max_range=130;
};



#endif // POINTCLOUDBAGREADER_H
