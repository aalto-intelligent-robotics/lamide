#ifndef LIDAR_UTILITIES_H
#define LIDAR_UTILITIES_H

#include <laser_geometry/laser_geometry.h>
//#include "gnuplot-iostream.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "strings.h"

namespace perception_oru
{
namespace graph_map
{
using std::cerr;
using std::cout;
using std::endl;
typedef std::vector<std::pair<double, double>> pair_vector_double;

class ScanPlot
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef enum scanRepresentation
    {
        RangeAngle = 0,
        XY = 1,
        RangeAngleDerivative = 3,
        XYDerivative = 4
    } scanRepresentation;
    void expSmoothO1(pair_vector_double& data, double alpha);
    void scanToPairVector(const sensor_msgs::LaserScan& scan,
                          const scanRepresentation rep,
                          pair_vector_double& data);
    void plot(const sensor_msgs::LaserScan& scan, const scanRepresentation plot_type, double alpha);
    void plot(const sensor_msgs::LaserScan& scan, const scanRepresentation plot_type);

private:
    void plot(const sensor_msgs::LaserScan& scan,
              const scanRepresentation plot_type,
              const std::string PlotOptsX,
              const std::string PlotOptsY,
              double alpha);
    void derivative(pair_vector_double& data_in, pair_vector_double& derivative);
    // Gnuplot gp;
};
} // namespace graph_map
} // namespace perception_oru
#endif // LIDAR_UTILITIES_H
