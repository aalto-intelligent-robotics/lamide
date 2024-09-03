#include "ndt_localization_lamide/helper_functions.hpp"

void to2PI(double &a){
  a = (double)fmod((double)(a), (double)( 2 * M_PI));
  if(a < 0) a += 2 * (double)M_PI;
}

void toPI(double &a){
  if(a > M_PI)
    while(a > M_PI) a -= 2.0 * M_PI;
  else
    if(a < -M_PI)
      while(a < -M_PI) a += 2.0 * M_PI;
}

void EigenSort( Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors ){
  int k, j, i;
  double p;
  for(i = 0; i < 2; i++){
    p = eigenvalues(k = i);
    for(j = i + 1; j < 3; j++)
      if(fabs(eigenvalues(j)) >= fabs(p))
        p = eigenvalues(k = j);
    if(k != i){
      eigenvalues.row(k).swap(eigenvalues.row(i));
      eigenvectors.col(k).swap(eigenvectors.col(i));
    }
  }
}

void EigenSort2D( Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors ){
  Eigen::Vector3d ez;
  ez << 0.0, 0.0, 1.0;
  double dist = 0;
  int z_id;
  Eigen::Vector3d normal_vec;
  Eigen::Vector3d tangent_vec;
  Eigen::Vector3d xy_vec;
  double normal_val;
  double tangent_val;
  double xy_val;
  for(int i = 0; i < 3; ++i){
    if(fabs(eigenvectors.col(i).dot(ez)) > dist){
      z_id = i;
      dist = fabs(eigenvectors.col(i).dot(ez));
    }
  }
  xy_vec=eigenvectors.col(z_id);
  xy_val=eigenvalues(z_id);
  if(z_id == 0){
    normal_vec = eigenvectors.col(1);
    tangent_vec = eigenvectors.col(2);
    normal_val = eigenvalues(1);
    tangent_val = eigenvalues(2);
  }
  if(z_id == 1){
    normal_vec = eigenvectors.col(0);
    tangent_vec = eigenvectors.col(2);
    normal_val = eigenvalues(0);
    tangent_val = eigenvalues(2);
  }
  if(z_id == 2){
    normal_vec = eigenvectors.col(0);
    tangent_vec = eigenvectors.col(1);
    normal_val = eigenvalues(0);
    tangent_val = eigenvalues(1);
  }
  if(normal_val > tangent_val){
    double temp_val=normal_val;
    normal_val=tangent_val;
    tangent_val=temp_val;
    Eigen::Vector3d temp_vec=normal_vec;
    normal_vec=tangent_vec;
    tangent_vec=temp_vec;
  }

  eigenvectors.col(0)=tangent_vec;
  eigenvalues(0)=tangent_val;
  eigenvectors.col(1)=normal_vec;
  eigenvalues(1)=normal_val;
  eigenvectors.col(2)=xy_vec;
  eigenvalues(2)=xy_val;

}


Eigen::Affine3d getAsAffine(float x, float y, float yaw ){
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
	    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
	    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x, y, 0);
	Eigen::Affine3d T = v * m;
	return T;
}
