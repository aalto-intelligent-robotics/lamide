#ifndef MCL_NDT_LAMIDE_H
#define MCL_NDT_LAMIDE_H
#include "graph_localization_lamide/localization_factory.h" //must be included first
#include "graph_localization_lamide/localization_type.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt_utils.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt.h"
#include "ndt_generic_lamide/eigen_utils.h"
#include "ndt_generic_lamide/motion_model_3d.h"
#include "ndt_localization_lamide/3d_ndt_mcl.hpp"
#include "ndt_localization_lamide/3d_particle_filter.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "stdio.h"
#include "time.h"
#include "vector"

#include <angles/angles.h>
#include <ndt_generic_lamide/point_types.h>

namespace perception_oru
{
namespace graph_localization
{

class ClassWeights
{
public:
    void set(int label1, int label2, double weight)
    {
        data_[label1][label2] = weight;
        count_++;
    }

    double get(int label1, int label2)
    {
        return data_[label1][label2];
    }

    void print()
    {
        std::cout << count_ << " class weights initialized" << std::endl;
    }

private:
    std::map<int, std::map<int, double>> data_;
    int count_ = 0;
};

class MCLNDTLamideType : public MCLNDTType
{
public:
    MCLNDTLamideType(LocalisationParamPtr param);

protected:
    void AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts) override;

    void parseWeights(const std::string& path);

private:
    ClassWeights weights_;
};

class MCLNDTLamideParam : public MCLNDTParam
{
public:
    void GetParamFromRos() override;
    std::string weights_path_ = "";
};

} // namespace graph_localization
} // namespace perception_oru
#endif // MCL_NDT_H
