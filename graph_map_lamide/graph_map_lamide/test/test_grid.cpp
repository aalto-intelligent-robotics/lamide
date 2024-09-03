#include "Eigen/Geometry"
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graph_plot.h"
#include "graph_map_lamide/graphfactory.h"
#include "graph_map_lamide/voxelgrid.h"
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "stdio.h"

#include <pcl/point_cloud.h>

using namespace std;
using namespace Eigen;

using namespace perception_oru;
using namespace graph_map;
class TestVoxelGrid
{
public:
    TestVoxelGrid()
    {
    }

    void TestIndex()
    {
        Eigen::Vector3d resolution(2, 2, 2);
        Eigen::Vector3d center(1, 1, 1);
        VoxelGrid<int> grid(resolution, center);

        for (int i = -10; i < 10; i++)
        {
            cout << std::endl;
            Eigen::Vector3d pos(i + 0.00001, i + 0.000001, i + 0.00001);
            cout << "input=" << pos(0) << std::endl;
            Eigen::Vector3i idx = grid.GetIndex(pos);
            cout << "Cell index " << idx << endl;
        }
    }

    void TestStoreData()
    {
        Eigen::Vector3d resolution(1, 1, 1);
        Eigen::Vector3d center(0, 0, 0);
        VoxelGrid<std::string> grid(resolution, center);
        Eigen::Vector3d p1(0.1, 0.1, 0.1);
        Eigen::Vector3d p2(1.1, 1.1, 1.1);
        grid.SetVal("text1", p1);
        grid.SetVal("text2", p2);
        std::string text;
        grid.GetVal(text, p1);
        cout << "read the text: " << text << std::endl;
    }
};

int main(int argc, char** argv)
{

    TestVoxelGrid test;
    test.TestStoreData();
}
