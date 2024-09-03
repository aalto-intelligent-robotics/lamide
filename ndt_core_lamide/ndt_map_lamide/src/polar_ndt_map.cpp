#include <ndt_map_lamide/polar_ndt_map.h>
namespace perception_oru{
PolarNDTMap::PolarNDTMap(double resolution_y_deg, double resolution_p_deg, double resolution_r) : nh_("~") {
  resolution_p_=resolution_p_deg*M_PI/180.0;;
  resolution_y_=resolution_y_deg*M_PI/180.0;
  resolution_r_=resolution_r;

  size_y_=ceil(2*M_PI/resolution_y_)+1;
  size_p_=ceil(2*M_PI/resolution_p_)+1;
  size_r_=ceil(150.0/resolution_r)+1;

  v_tolerance=0.1*M_PI/18.00;

  pub=new ros::Publisher();
  *pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZL> >("/points2", 10);
}
void PolarNDTMap::AllocateMemory(){
  data = new pcl::PointCloud<pcl::PointXYZL> **[size_y_];
  for (int i = 0; i < size_y_; ++i) {
    data[i] = new pcl::PointCloud<pcl::PointXYZL>*[size_p_];
    for (int j = 0; j < size_p_; ++j)
      data[i][j] = new pcl::PointCloud<pcl::PointXYZL> [size_r_];
  }

  /* cov_data = new pcl::PointCloud<pcl::PointXYZL> **[size_y_];
  for (int i = 0; i < size_y_; ++i) {
    cov_data[i] = new pcl::PointCloud<pcl::PointXYZL>*[size_p_];
    for (int j = 0; j < size_p_; ++j)
      cov_data[i][j] = new pcl::PointCloud<pcl::PointXYZL> [size_r_];
  }*/
}
void PolarNDTMap::DeallocateMemory(){
  // De-Allocate memory to prevent memory leak
  for (int i = 0; i < size_y_; ++i) {
    for (int j = 0; j < size_p_; ++j)
      delete [] data[i][j];

    delete [] data[i];
  }
  delete [] data;
}
void PolarNDTMap::GetCloud( double yaw_min, double yaw_max, pcl::PointCloud<pcl::PointXYZL> &cloud){

  for(int i=YawToIndex(yaw_min);i<=YawToIndex(yaw_max);i++){
  for(int j=0;j<size_p_;j++){
      for(int k=0;k<size_r_;k++){
        if(data[i][j][k].size()>0){
        std::cout<<"Add data size"<<data[i][j][k].size()<<std::endl;
        cloud+=data[i][j][k];
        }

      }
    }
  }
}
unsigned int PolarNDTMap::YawToIndex(double yaw) {return nearbyint(yaw/resolution_y_);}
void PolarNDTMap::AddPointCloud(pcl::PointCloud<pcl::PointXYZL> &cloud){
  /* try {
    DeallocateMemory();
  } catch (...) {
  }*/
  AllocateMemory();
  char c;
  ros::Time t=ros::Time::now();
  SegmentByDiscretisation(cloud);
  std::cout<<"time="<<ros::Time::now()-t<<std::endl;

  std::cin>>c;
  std::cout<<"Inserted pointcloud"<<std::endl;
}
void PolarNDTMap::SegmentByDiscretisation(pcl::PointCloud<pcl::PointXYZL> &cloud){
  for(int i=0;i<cloud.size();i++){
    pcl::PointXYZL p;
    p=cloud[i];
    double y_angle =atan2(p.y,p.x)+M_PI;//0-2*PI
    double r=sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    double p_angle=asin(p.z/r)+M_PI; //0-2*PI


    unsigned int r_idx=nearbyint(r/resolution_r_);
    unsigned int p_idx=nearbyint(p_angle/resolution_p_);
    unsigned int y_idx=nearbyint(y_angle/resolution_y_);

    //std::cout<<"size_r_="<<size_r_<<", size_p_="<<size_p_<<", size_y_="<<size_y_<<std::endl;
    //std::cout<<"r ="<<r_idx<<", p_idx="<<p_idx<<", y_idx="<<y_idx<<std::endl;
    data[y_idx][p_idx][r_idx].push_back(p);
  }
}
void PolarNDTMap::SegmentByRings(pcl::PointCloud<pcl::PointXYZL> &cloud){

  std::vector<double> rings;
  ros::Time T=ros::Time::now();
  for(int i=0;i<cloud.size();i++){
    pcl::PointXYZL p;
    p=cloud[i];
    float y_angle = atan2(p.y,p.x)+M_PI;//0-2*PI
    double r = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    float p_angle=asin(p.z/r)+M_PI; //0-2*PI


    unsigned int r_idx=nearbyint(r/resolution_r_);
    unsigned int y_idx=nearbyint(y_angle/resolution_y_);
    unsigned int p_idx=0;
    bool ring_found=false;
    for(int i=0;i<rings.size();i++){
      if( similar(p_angle,rings[i]) ){
        p_idx=i;
        ring_found=true;
        break;
      }
    }
    if(!ring_found)
      rings.push_back(p_angle);

    data[y_idx][p_idx][r_idx].push_back(p);
  }
  std::cout<<"Found "<<rings.size()<<" rings"<<std::endl;
  std::cout<<"Time="<<ros::Time::now()-T<<std::endl;
}

void PolarNDTMap::PlotSegments(){

  for(int i=0;i<size_y_;i++){
    for(int j=0;j<size_p_;j++){
      for(int k=0;k<size_r_;k++){
        pcl::PointCloud<pcl::PointXYZL> cloud=data[i][j][k];
        cloud.header.frame_id="world";
        pcl_conversions::toPCL(ros::Time::now(),cloud.header.stamp);
        cloud.height=1;
        cloud.width=cloud.size();
        if(cloud.size()>0){
          pub->publish(cloud);
          usleep(200000);
        }
        if(!ros::ok())
          exit(0);
      }
    }
  }

}
void PolarNDTMap::ComputeAllCells(std::vector<NDTCell*> &cells){

  for(int i=0;i<size_y_;i++){
    for(int j=0;j<size_p_;j++){
      for(int k=0;k<size_r_;k++){
        if(data[i][j][k].size()>4){
          Eigen::MatrixXd cov;
          Eigen::Vector3d mean;
          ComputeMean(mean,data[i][j][k]);
          ComputeCellCovariance(cov,data[i][j][k],mean);
          NDTCell * cell=new NDTCell();
          cell->setCov(cov);
          cell->setMean(mean);
          cell->setOccupancy(200);
          cell->hasGaussian_=true;
          pcl::PointXYZL p;
          p.x=mean(0);
          p.y=mean(1);
          p.z=mean(2);
          cell->setCenter(p);
          cells.push_back(cell);
        }
      }
    }
  }
  std::cout<<"Computer number of cells="<<cells.size()<<std::endl;


}
void PolarNDTMap::ComputeCellCovariance(Eigen::MatrixXd &cov, const pcl::PointCloud<pcl::PointXYZL> &segment, const Eigen::Vector3d &mean){

  Eigen::MatrixXd mp;
  mp.resize(segment.size(),3);
  for(unsigned int i=0; i< segment.size(); i++){
    mp(i,0) = segment[i].x - mean(0);
    mp(i,1) = segment[i].y - mean(1);
    mp(i,2) = segment[i].z - mean(2);
  }
  Eigen::Matrix3d covSum = mp.transpose()*mp;
  cov = covSum/(segment.size()-1);

}
void PolarNDTMap::ComputeMean(Eigen::Vector3d &mean, const pcl::PointCloud<pcl::PointXYZL> &segment){
  double x,y,z;
  unsigned int N=segment.size();
  if(N<4)
    return;

  for(int i=0;i<segment.size();i++){
    x+=segment[i].x;
    y+=segment[i].y;
    z+=segment[i].z;
  }
  x=x/N;
  y=y/N;
  z=z/N;
  mean<<x,y,z;
}

}
