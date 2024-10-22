#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/lazy_grid.h>
#include <pointcloud_vrml/pointcloud_utils.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>

// #include <opencv/cv.h>
// #include <opencv/highgui.h>
// #include <boost/shared_ptr.hpp>
// #include <boost/thread/mutex.hpp>
// #include <cv_bridge/CvBridge.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>

#include <Eigen/Eigen>

using namespace std;

int main (int argc, char** argv)
{
    if(argc < 2)
    {
        cout << "[ USAGE ] jffSaveTest cloud\n";
        exit(1);
    }

    cout << "Started saveTest.\n";

    pcl::PointCloud<pcl::PointXYZL> cloud;
    char fname[] = "test_wrl.wrl";

    //cloud = lslgeneric::readVRML<pcl::PointXYZL>(argv[1]);

    perception_oru::NDTMap<pcl::PointXYZL> nd(new perception_oru::LazyGrid<pcl::PointXYZL>(0.2));

    nd.loadPointCloud(cloud);
    nd.computeNDTCells();

    //nd.writeToVRML(fname);

    if (nd.writeToJFF("LazyGrid.jff") < 0)
        cout << "writing to jff failed\n";

    cout << "Finished saveTest.\n";

    return 0;
}

