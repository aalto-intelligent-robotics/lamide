#ifndef POLARDISCRETIZEDNDTMAP_H
#define POLARDISCRETIZEDNDTMAP_H
#include "ndt_cell.h"
#include "ndt_map.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/node_handle.h"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "ros/time.h"
#include "parallel/parallel.h"
namespace perception_oru {
typedef enum Segmentationtype {even_discretisation=0, rings=1}cloud_segmentaiton;

class PolarNDTMap
{
public:
//PolarNDTMap(double resolution_y_deg=2., double resolution_p_deg=2.7, double resolution_r=1.5);
  PolarNDTMap(double resolution_y_deg=16, double resolution_p_deg=7, double resolution_r=0.6);

  ~PolarNDTMap(){DeallocateMemory();}


  void AddPointCloud(pcl::PointCloud<pcl::PointXYZL> &cloud);

  void PlotSegments();

  void AllocateMemory();

  void DeallocateMemory();

  void ComputeAllCells(std::vector<NDTCell*> &cells);

  void GetCloud(double yaw_min, double yaw_max, pcl::PointCloud<pcl::PointXYZL> &cloud);

  unsigned int YawToIndex(double yaw);

private:

  void SegmentByRings(pcl::PointCloud<pcl::PointXYZL> &cloud);

  void SegmentByDiscretisation(pcl::PointCloud<pcl::PointXYZL> &cloud);

  void ComputeMean(Eigen::Vector3d &mean, const pcl::PointCloud<pcl::PointXYZL> &segment);

  void ComputeCellCovariance(Eigen::MatrixXd &cov, const pcl::PointCloud<pcl::PointXYZL> &segment, const Eigen::Vector3d &mean);

  inline bool similar(float x, float y)const{  return(fabs(y-x)<v_tolerance); }



int size_y_,size_p_,size_r_;
double resolution_r_,resolution_y_,resolution_p_;
pcl::PointCloud<pcl::PointXYZL> ***data;
NDTCell*  ***cov_data;
std::vector<std::vector<std::vector<double> > > data2;
ros::Publisher *pub;
ros::NodeHandle nh_;
ros::Publisher *cloud_pub;
double v_tolerance; //0.2 degrees


};

//std::vector<std::vector>

}
#endif // POLARDISCRETIZEDNDTMAP_H
