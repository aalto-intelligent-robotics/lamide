#include "graph_map_lamide/utils.h"

void printPose(const Eigen::Affine3d& p, const std::string& name)
{
    std::cout << name << ": " << p(0, 3) << " " << p(1, 3) << " " << p(2, 3) << std::endl;
}