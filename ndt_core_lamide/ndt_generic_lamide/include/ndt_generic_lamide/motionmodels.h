#ifndef NDT_GENERIC_MOTIONMODELS_H
#define NDT_GENERIC_MOTIONMODELS_H
#include "ndt_generic_lamide/motion_model_3d.h"
// Until ported this to YAML file
namespace perception_oru
{
#if 0
inline bool GetMotionModel(const std::string &dataset, std::vector<double> &motion_model, std::vector<double> &motion_model_offset){

 if(dataset.compare("hx")==0 || dataset.compare("volvo_2017_12_01")==0){
   std::cout<<"hx motion settings applied"<<std::endl;

   motion_model.clear();

   motion_model.push_back(0.01);
   motion_model.push_back(0.002);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.005);
   motion_model.push_back(0.001);

   motion_model.push_back(0.002);
   motion_model.push_back(0.005);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.005);

   motion_model.push_back(0.005);
   motion_model.push_back(0.001);
   motion_model.push_back(0.01);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.005);

   motion_model.push_back(0.002);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.01);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);

   motion_model.push_back(0.005);
   motion_model.push_back(0.002);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.04);
   motion_model.push_back(0.001);

   motion_model.push_back(0.005);
   motion_model.push_back(0.002);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.01);

   motion_model_offset.push_back(0.01);
   motion_model_offset.push_back(0.01);
   motion_model_offset.push_back(0.01);
   motion_model_offset.push_back(0.001);
   motion_model_offset.push_back(0.002);
   motion_model_offset.push_back(0.001);
 }
 else if(dataset.compare("arla-2012")==0){
   std::cout<<"arla-2012 motion settings applied"<<std::endl;
     motion_model.clear();
     motion_model.push_back(0.05);
     motion_model.push_back(0.05);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.04);

     motion_model.push_back(0.05);
     motion_model.push_back(0.1);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.04);


     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.01);
     motion_model.push_back(0.01);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.02);

     motion_model_offset.push_back(0.0005);
     motion_model_offset.push_back(0.0005);
     motion_model_offset.push_back(0.000000000);//0.002
     motion_model_offset.push_back(0.0000000);//0.001
     motion_model_offset.push_back(0.0000000);//0.001
   motion_model_offset.push_back(0.0003);

 }

}
#endif

bool GetMotionModel(const std::string& dataset, MotionModel3d& motionModel)
{
    MotionModel3d::Params par;
    if (dataset == "arla-2012")
    {
        par.set2DParams(0.5, 0.5, 1.0, 2, 0.3, 2.5);
        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.01, 0.01, 0.000, 0.000, 0.000, 0.02;
        par.SetOffset(offset);
    }
    else if (dataset == "arla-2012-noisy")
    {
        par.set2DParams(5, 2, 5, 2, 2.0, 2.5);
        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.1, 0.1, 0.000, 0.000, 0.000, 0.05;
        par.SetOffset(offset);
    }
    else if (dataset == "ncfm-2018")
    {
        par.set2DParams(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.01, 0.01, 0.000, 0.000, 0.000, 0.01;
        par.SetOffset(offset);
    }
    if (dataset == "orkla-velodyne")
    {

        par.set3DParams(1, 3,      // x
                        1, 3,      // y
                        0.5, 0.4,  // z
                        0.1, 0.01, // ex
                        0.2, 0.1,  // ey
                        0.2, 2);   // ez

        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.02, 0.02, 0.003, 0.001, 0.001, 0.01;
        par.SetOffset(offset);
    }
    else if (dataset == "orkla-citi-2019")
    {
        par.set2DParams(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.01, 0.01, 0.000, 0.000, 0.000, 0.01;
        par.SetOffset(offset);
    }
    else if (dataset == "michigan")
    {
        par.set3DParams(4, 0.12,    // x
                        4, 0.12,    // y
                        1.0, 0.4,   // z
                        0.01, 0.01, // ex
                        0.01, 0.01, // ey
                        0.2, 0.1);  // ez
        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.003, 0.003, 0.003, 0.001, 0.001, 0.001;
        par.SetOffset(offset);
        std::cout << "Using motion model \"michigan\"" << std::endl;
    }
    else if (dataset == "michigan-noisy")
    {
        par.set3DParams(13, 0.45,   // x
                        18, 0.45,   // y
                        4.5, 0.27,  // z
                        0.8, 0.45,  // ex
                        0.8, 0.45,  // ey
                        0.8, 0.20); // ez
        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.4, 0.4, 0.020, 0.01, 0.01, 0.01;
        par.SetOffset(offset);
        std::cout << "Using motion model \"michigan\"" << std::endl;
    }
    else if (dataset == "volvo_2017_12_01" || dataset.compare("hx") == 0)
    {
        par.set3DParams(0.08, 0.0,   // x
                        0.08, 0.08,  // y
                        0.16, 0.0,   // z
                        0.03, 0.0,   // ex
                        0.05, 0.0,   // ey
                        0.08, 0.08); // ez

        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.003, 0.003, 0.003, 0.001, 0.001, 0.001;
        par.SetOffset(offset);
    }
    else if (dataset == "lamide")
    {
        par.set3DParams(50, 20,
                        50, 20,
                        20, 25,
                        5, 2.5, // ex
                        5, 2.5,                      // ey
                        5, 2.5);                    // ez

        Eigen::Matrix<double, 6, 1> offset;
        offset << 0.4, 0.4, 0.020, 0.01, 0.01, 0.01;
        par.SetOffset(offset);
    }
    motionModel.setParams(par);
    return true;
}
} // namespace perception_oru
#endif // MOTIONMODELS_H
