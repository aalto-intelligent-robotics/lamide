#include "ndt_localization_lamide/ndt_histogram_2D.hpp"
#include "ros/ros.h"
#include <fstream>

perception_oru::histogram_cell::histogram_cell(double te_,
                                               int ns_,
                                               int np_,
                                               std::vector<double> begins_,
                                               std::vector<double> ends_,
                                               double cs_,
                                               Eigen::Vector2d origin_,
                                               double X,
                                               double Y,
                                               int os_,
                                               double opening_)
    : histogram_definition(te_, ns_, np_, begins_, ends_, cs_, origin_, os_, opening_)
    , contributing_cells(0)
{
    // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    cX = X;
    cY = Y;
    histograms.resize(os);
    for (int i = 0; i < os; ++i)
        histograms[i].setSize(np, ns, begins.size());
}

perception_oru::histogram_cell::histogram_cell(const std::shared_ptr<NDTMap>& map_,
                                               double te_,
                                               int ns_,
                                               int np_,
                                               std::vector<double> begins_,
                                               std::vector<double> ends_,
                                               double cs_,
                                               Eigen::Vector2d origin_,
                                               int os_,
                                               double opening_)
    : histogram_definition(te_, ns_, np_, begins_, ends_, cs_, origin_, os_, opening_)
    , contributing_cells(0)
{
    // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    histograms.resize(os);
    for (int i = 0; i < os; ++i)
        histograms[i].setSize(np, ns, begins.size());
    std::vector<perception_oru::NDTCell*> allCells = map_->getAllInitializedCells();
    for (int cInd = 0; cInd < allCells.size(); cInd++)
        if (allCells[cInd]->hasGaussian_)
            contributing_cells.push_back(allCells[cInd]);

    compute_histogram();
    //  for(size_t i = 0; i < allCells.size(); i++)
    //    delete allCells[i];
    // for (size_t i = 0; i <  contribuhting_cells.size(); i++) {
    //  delete contributing_cells[i];
    // }
}

std::vector<int> perception_oru::histogram_cell::getDistBin(Eigen::Vector2d center)
{
    // double dist = (origin - center).norm();
    double dist = center.norm();

    // std::cout << dist << ",\n";
    std::vector<int> res;
    for (int i = 0; i < begins.size(); ++i)
        if (begins[i] <= dist && ends[i] > dist)
            res.push_back(i);
    if (res.size() == 0)
        res.push_back(-1);
    return res;
}

double perception_oru::histogram_cell::score(perception_oru::histogram_cell outer)
{
    std::vector<std::vector<int>> outer_hist;
    outer_hist = outer.histograms[0].sphere_classes;
    outer_hist.insert(outer_hist.end(), outer.histograms[0].planar_classes.begin(),
                      outer.histograms[0].planar_classes.end());
    int n1_outer = 0;
    double best_score = std::numeric_limits<double>::max();
    if (opening != 2 * M_PI)
    { // for NON 360 sensors
        for (int i = 0; i < ns + np; ++i)
            n1_outer += std::accumulate(outer_hist[i].begin(), outer_hist[i].end(), 0);
        for (int j = 0; j < os; ++j)
        {
            std::vector<std::vector<int>> map_hist;
            int n1_map = 0;
            map_hist = histograms[j].sphere_classes;
            map_hist.insert(map_hist.end(), histograms[j].planar_classes.begin(),
                            histograms[j].planar_classes.end());
            for (int i = 0; i < ns + np; ++i)
                n1_map += std::accumulate(map_hist[i].begin(), map_hist[i].end(), 0);

            double score = compare(map_hist, n1_map, outer_hist, n1_outer);
            if (score < best_score)
            {
                best_score = score;
                cell_orientation = 2.0 * M_PI * (double(j) / double(os));
            }
        }
    }
    else
    { // for 360 sensors
        for (int i = 0; i < ns + np; ++i)
            n1_outer += std::accumulate(outer_hist[i].begin(), outer_hist[i].end(), 0);
        for (int j = 0; j < np; ++j)
        {
            std::vector<std::vector<int>> map_hist;
            int n1_map = 0;
            // map_hist = histograms[0].sphere_classes;
            for (int i = 0; i < ns; ++i)
                map_hist.push_back(histograms[0].sphere_classes[i]);
            for (int i = np - j; i < np; ++i)
                map_hist.push_back(histograms[0].planar_classes[i]);
            for (int i = 0; i < np - j; ++i)
                map_hist.push_back(histograms[0].planar_classes[i]);
            for (int i = 0; i < ns + np; ++i)
                for (int j = 0; j < begins.size(); ++j)
                {
                    // std::cout << __LINE__<<":"<<i<<":"<<j<<":"<<":"<<map_hist.size()<<":"<<
                    // map_hist[i][j] << "\n";
                    n1_map += map_hist[i][j];
                }
            // n1_map += std::accumulate(map_hist[i].begin(), map_hist[i].end(), 0);
            double score = compare(map_hist, n1_map, outer_hist, n1_outer);
            if (score < best_score)
            {
                best_score = score;
                cell_orientation = 2.0 * M_PI * (double(j) / double(np));
            }
        }
    }
    cell_score = best_score;
    return best_score;
}

double perception_oru::histogram_cell::scoreNoChange(perception_oru::histogram_cell outer)
{
    std::vector<std::vector<int>> outer_hist;
    outer_hist = outer.histograms[0].sphere_classes;
    outer_hist.insert(outer_hist.end(), outer.histograms[0].planar_classes.begin(),
                      outer.histograms[0].planar_classes.end());
    int n1_outer = 0;
    double best_score = std::numeric_limits<double>::max();
    if (opening != 2 * M_PI)
    { // for NON 360 sensors
        for (int i = 0; i < ns + np; ++i)
            n1_outer += std::accumulate(outer_hist[i].begin(), outer_hist[i].end(), 0);
        for (int j = 0; j < os; ++j)
        {
            std::vector<std::vector<int>> map_hist;
            int n1_map = 0;
            map_hist = histograms[j].sphere_classes;
            map_hist.insert(map_hist.end(), histograms[j].planar_classes.begin(),
                            histograms[j].planar_classes.end());
            for (int i = 0; i < ns + np; ++i)
                n1_map += std::accumulate(map_hist[i].begin(), map_hist[i].end(), 0);

            double score = compare(map_hist, n1_map, outer_hist, n1_outer);
            if (score < best_score)
            {
                best_score = score;
            }
        }
    }
    else
    { // for 360 sensors
        for (int i = 0; i < ns + np; ++i)
            n1_outer += std::accumulate(outer_hist[i].begin(), outer_hist[i].end(), 0);
        for (int j = 0; j < np; ++j)
        {
            std::vector<std::vector<int>> map_hist;
            int n1_map = 0;
            // map_hist = histograms[0].sphere_classes;
            for (int i = 0; i < ns; ++i)
                map_hist.push_back(histograms[0].sphere_classes[i]);
            for (int i = np - j; i < np; ++i)
                map_hist.push_back(histograms[0].planar_classes[i]);
            for (int i = 0; i < np - j; ++i)
                map_hist.push_back(histograms[0].planar_classes[i]);
            for (int i = 0; i < ns + np; ++i)
                for (int j = 0; j < begins.size(); ++j)
                {
                    // std::cout << __LINE__<<":"<<i<<":"<<j<<":"<<":"<<map_hist.size()<<":"<<
                    // map_hist[i][j] << "\n";
                    n1_map += map_hist[i][j];
                }
            // n1_map += std::accumulate(map_hist[i].begin(), map_hist[i].end(), 0);
            double score = compare(map_hist, n1_map, outer_hist, n1_outer);
            if (score < best_score)
            {
                best_score = score;
            }
        }
    }
    return best_score;
}

double perception_oru::histogram_cell::compare(std::vector<std::vector<int>> F,
                                               int n1_F,
                                               std::vector<std::vector<int>> G,
                                               int n1_G)
{
    double ratio = std::max(double(n1_F), double(n1_G)) / std::min(double(n1_F), double(n1_G));
    double score = 0;

    for (int i = 0; i < ns + np; ++i)
    {
        // for(int i = 0; i < np; ++i){
        double n2_row = 0;
        for (int j = 0; j < begins.size(); ++j)
            n2_row += ((double(F[i][j]) / double(n1_F) - double(G[i][j]) / double(n1_G)) *
                       (double(F[i][j]) / double(n1_F) - double(G[i][j]) / double(n1_G)));

        score += sqrt(n2_row);
    }
    return score; //* ratio;
}

bool perception_oru::histogram_cell::compute_histogram()
{
    for (int i = 0; i < contributing_cells.size(); ++i)
    {
        Eigen::Matrix3d Cevec = contributing_cells[i]->getEvecs();
        Eigen::Vector3d Ceval = contributing_cells[i]->getEvals();
        ////////////////////////////////////////////////////////////////////////
        // check wich of the vectors is perpendicular to plane XY
        // check which one is normal (the shortest from the remining ones)
        Eigen::Vector3d ez;
        ez << 0.0, 0.0, 1.0;
        double dist = 0;
        int z_id;
        Eigen::Vector3d normal_vec;
        Eigen::Vector3d tangent_vec;
        double normal_val;
        double tangent_val;
        for (int i = 0; i < 3; ++i)
        {
            if (fabs(Cevec.col(i).dot(ez)) > dist)
            {
                z_id = i;
                dist = fabs(Cevec.col(i).dot(ez));
            }
        }
        if (z_id == 0)
        {
            normal_vec = Cevec.col(1);
            tangent_vec = Cevec.col(2);
            normal_val = Ceval(1);
            tangent_val = Ceval(2);
        }
        if (z_id == 1)
        {
            normal_vec = Cevec.col(0);
            tangent_vec = Cevec.col(2);
            normal_val = Ceval(0);
            tangent_val = Ceval(2);
        }
        if (z_id == 2)
        {
            normal_vec = Cevec.col(0);
            tangent_vec = Cevec.col(1);
            normal_val = Ceval(0);
            tangent_val = Ceval(1);
        }
        if (normal_val > tangent_val)
        {
            double temp_val = normal_val;
            normal_val = tangent_val;
            tangent_val = temp_val;
            Eigen::Vector3d temp_vec = normal_vec;
            normal_vec = tangent_vec;
            tangent_vec = temp_vec;
        }

        ////////////////////////////////////////////////////////////////////////
        // EigenSort(Ceval, Cevec);
        Eigen::Vector3d center3d = contributing_cells[i]->getMean();
        Eigen::Vector2d center;
        center << center3d[0], center3d[1];
        // std::cout << "center="<< center.transpose() << "\n";
        center = center - origin;
        std::vector<int> did = getDistBin(center);
        // for(int dd = 0; dd < did.size(); ++dd)
        //     std::cout << "distance bucket="<<did[dd] << "\n";
        center.normalize();
        if (opening != 2 * M_PI)
        {
            for (int j = 0; j < os; ++j)
            {
                double cent_angle = 2.0 * M_PI * (double(j) / double(os));
                Eigen::Vector2d cent_vec;
                cent_vec << cos(cent_angle), sin(cent_angle);
                double ang = acos(cent_vec.dot(center));
                if (ang <= opening / 2.0 || opening < 0.0)
                {
                    histograms[j].contributing_cells.push_back(contributing_cells[i]);
                    if (normal_val / tangent_val >= te)
                    { // evaluate if distribution circular enough

                        int sid = std::max(int(ceil(ns * tangent_val / cs)), ns) - 1;
                        for (int dd = 0; dd < did.size(); ++dd)
                            histograms[j].sphere_classes[sid][did[dd]]++;
                        // we are spliting circular classes based on size (comapred againsta cell
                        // size)
                    }
                    else
                    {

                        Eigen::Vector2d orientation;
                        Eigen::Matrix2d rot;
                        rot << cos(cent_angle), -sin(cent_angle), sin(cent_angle), cos(cent_angle);
                        // orientation << Cevec.col(1)[0], Cevec.col(1)[1];
                        orientation << normal_vec[0], normal_vec[1];
                        // std::cout << orientation.transpose() << "\n";
                        orientation = orientation.transpose() * rot;
                        Eigen::Vector2d center_l = center.transpose() * rot;
                        // std::cout << orientation.transpose() << "\n";
                        int oid = getOrientationBin(orientation, center_l);
                        // std::cout << "orinetation bucket =" << oid<< "\n";
                        // Eigen::Vector3d center3d =  contributing_cells[i]->getMean();
                        // Eigen::Vector2d center;
                        // center << center3d[0], center3d[1];
                        // std::vector<int> did = getDistBin(center);
                        for (int dd = 0; dd < did.size(); ++dd)
                            if (did[dd] != -1)

                                histograms[j].planar_classes[oid][did[dd]]++;
                    }
                }
            }
        }
        else
        {
            double cent_angle = 0.0;
            Eigen::Vector2d cent_vec;
            cent_vec << cos(cent_angle), sin(cent_angle);
            double ang = acos(cent_vec.dot(center));

            histograms[0].contributing_cells.push_back(contributing_cells[i]);
            if (normal_val / tangent_val >= te)
            { // evaluate if distribution circular enough
                int sid = std::max(int(ceil(ns * tangent_val / cs)), ns) - 1;
                for (int dd = 0; dd < did.size(); ++dd)
                    histograms[0].sphere_classes[sid][did[dd]]++;
                // we are spliting circular classes based on size (comapred againsta cell size)
            }
            else
            {
                Eigen::Vector2d orientation;
                Eigen::Matrix2d rot;
                rot << cos(cent_angle), -sin(cent_angle), sin(cent_angle), cos(cent_angle);
                // orientation << Cevec.col(1)[0], Cevec.col(1)[1];
                orientation << normal_vec[0], normal_vec[1];
                // std::cout << orientation.transpose() << "\n";
                orientation = orientation.transpose() * rot;
                Eigen::Vector2d center_l = center.transpose() * rot;
                // std::cout << orientation.transpose() << "\n";
                int oid = getOrientationBin(orientation, center_l);
                // std::cout << "orinetation bucket =" << oid<< "\n";
                // Eigen::Vector3d center3d =  contributing_cells[i]->getMean();
                // Eigen::Vector2d center;
                // center << center3d[0], center3d[1];
                // std::vector<int> did = getDistBin(center);
                for (int dd = 0; dd < did.size(); ++dd)
                    if (did[dd] != -1)
                        histograms[0].planar_classes[oid][did[dd]]++;
            }
        }
    }
    return true;
}

int perception_oru::histogram_cell::getOrientationBin(Eigen::Vector2d direction,
                                                      Eigen::Vector2d center)
{
    double dist_min = std::numeric_limits<double>::min();
    int bucket;
    // std::cout << atan2(direction[1],direction[0]) << "\n";
    for (int t = 0; t < np / 2; ++t)
    {
        double omega = double(t) * theta;
        double dist = fabs(direction[0] * cos(omega) +
                           direction[1] * sin(omega)); // not a real distance just dot product
        if (dist > dist_min)
        {
            dist_min = dist;
            bucket = t;
        }
    }
    // std::cout << "bucket p="<< bucket<<" "<<double(bucket)*theta << "\n";
    Eigen::Vector2d bucket_direction;
    bucket_direction << cos(double(bucket) * theta), sin(double(bucket) * theta);
    if (bucket_direction.dot(center) >
        0) // if we point outwords from the center we have to point inwards
        bucket += np / 2;
    // std::cout << "bucket a="<< bucket<<" "<<double(bucket)*theta << "\n";
    return bucket;
}

perception_oru::histogram_cell::~histogram_cell()
{
    histograms.clear();
    // for (size_t i = 0;i < contributing_cells.size();i++) {
    //   delete contributing_cells[i];
    // }
    contributing_cells.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

perception_oru::histogram_definition::histogram_definition(double te_,
                                                           int ns_,
                                                           int np_,
                                                           std::vector<double> begins_,
                                                           std::vector<double> ends_,
                                                           double cs_,
                                                           Eigen::Vector2d origin_,
                                                           int os_,
                                                           double opening_)
{
    this->te = te_;
    this->ns = ns_;
    this->np = np_;
    // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    // ROS_INFO_STREAM(begins_.size()<<" "<<ends_.size());
    try
    {
        if (begins_.size() != ends_.size())
            throw;
    }
    catch (...)
    {
        ROS_ERROR_STREAM("The vecotrs defining the edges of bins are not of equall size");
    }
    // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    this->begins = begins_;
    this->ends = ends_;
    this->cs = cs_;
    this->origin = origin_;
    this->theta = (2 * M_PI) / double(this->np);
    this->os = os_;
    if (opening_ == 2 * M_PI)
        os = 1;
    this->opening = opening_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void perception_oru::histogram_map::getIndexForPoint(double X, double Y, int& indX, int& indY)
{
    indX = floor((X - centerX) / cellSize + 0.5) + sizeX / 2.0;
    indY = floor((Y - centerY) / cellSize + 0.5) + sizeY / 2.0;
}

perception_oru::histogram_map::histogram_map(const std::shared_ptr<NDTMap>& map_,
                                             double te_,
                                             int ns_,
                                             int np_,
                                             std::vector<double> begins_,
                                             std::vector<double> ends_,
                                             double range_,
                                             int os_,
                                             double opening_)
{
    // initialise parameters
    os = os_;
    opening = opening_;
    map = map_;
    map->getCentroid(centerX, centerY, centerZ);
    map->getGridSize(sizeX, sizeY, sizeZ);
    map->getCellSizeInMeters(cellSize, cellSize, cellSize);
    range = range_;
    // build empty grid for histograms
    data_grid.resize(sizeX);
    for (int x = 0; x < sizeX; ++x)
        data_grid[x].resize(sizeY, NULL);
    // generate beams
    alpha = 0.017453; // 1 deg          //2 * asin(cellSize / (2 * range));
    double ang = 0;
    double range_cells = range / cellSize;
    while (ang < 2 * M_PI)
    {
        beams.push_back(RayTrace(ang, range_cells));
        ang += alpha;
    }
    // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    // ROS_INFO_STREAM("DISTANCES FOR THE MAP");
    std::vector<perception_oru::NDTCell*> allCells = map->getAllInitializedCells();
    std::vector<perception_oru::NDTCell*> cells;
    for (int cInd = 0; cInd < allCells.size(); cInd++)
        if (allCells[cInd]->getOccupancy() < 0.0)
            cells.push_back(allCells[cInd]);
    data.reserve(cells.size());
    // collect contributing cells
    for (int cInd = 0; cInd < cells.size(); cInd++)
    { // we will not need this (I think) TZKR
        // get the center of empty cell
        double orX, orY, orZ;
        cells[cInd]->getCenter(orX, orY, orZ);
        // get address of empty cell in the grid (histogram griid and map grid are equal)
        int idX, idY;
        // create new empty histogram for cell idX idY
        Eigen::Vector2d ori;
        ori << orX, orY;
        this->getIndexForPoint(orX, orY, idX, idY);
        // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
        histogram_cell* new_hc = new histogram_cell(te_, ns_, np_, begins_, ends_, cellSize, ori,
                                                    orX, orY, os_, opening_);
        // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
        if (new_hc == NULL)
            ROS_ERROR_STREAM("EMPTY NEW CELL");
        data.push_back(new_hc);
        data_grid[idX][idY] = data.back();
        // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
        for (int bIt = 0; bIt < beams.size(); bIt++)
        {
            for (int cIt = 0; cIt < beams[bIt].size(); cIt++)
            {
                pcl::PointXYZL center;
                center.x = orX + double(beams[bIt][cIt][0]) * cellSize;
                center.y = orY + double(beams[bIt][cIt][1]) * cellSize;
                center.z = orZ;
                perception_oru::NDTCell* cell;
                if (map->getCellAtPoint(center, cell))
                {
                    if (cell->hasGaussian_)
                    {
                        bool add = true;
                        for (int chIt = 0; chIt < data_grid[idX][idY]->contributing_cells.size();
                             chIt++)
                        {
                            if (data_grid[idX][idY]->contributing_cells[chIt] == cell)
                            {
                                add = false;
                                break;
                            }
                        }
                        if (add)
                            data_grid[idX][idY]->contributing_cells.push_back(cell);
                        break;
                    }
                }
                else
                    break;
            }
        }
        // std::cout << "c{" << cInd + 1 << "}=[\n";
        data_grid[idX][idY]->compute_histogram();
        // std::cout << "];\n";
    }
    // ROS_INFO_STREAM("DISTANCES FOR THE MAP --------------------------------");
    // confusion_matrix();
}

perception_oru::beam perception_oru::histogram_map::RayTrace(double alpha, double range)
{
    toPI(alpha);
    double xincrement = sqrt(tan(alpha) * tan(alpha) + 1);
    double yincrement = sqrt(1 / (tan(alpha) * tan(alpha)) + 1);
    double xside = xincrement;
    double yside = alpha != 0 ? yincrement : std::numeric_limits<double>::max();

    double dist = 0;
    int x = 0;
    int y = 0;
    std::vector<std::vector<int>> pass;
    pass.emplace_back(std::vector<int>(2, 0));
    while (dist < range)
    {
        if (xside < yside)
        {
            xside += xincrement;
            x += sgn(cos(alpha));
            std::vector<int> t;
            t.push_back(x);
            t.push_back(y);
            pass.push_back(t);
            dist += xincrement;
        }
        else
        {
            yside += yincrement;
            y += sgn(sin(alpha));
            std::vector<int> t;
            t.push_back(x);
            t.push_back(y);
            dist += yincrement;
            pass.push_back(t);
        }
    }
    return pass;
}

void perception_oru::histogram_map::score_map(histogram_cell outer)
{
    for (int i = 0; i < data.size(); ++i)
        data[i]->score(outer);
}

std::vector<std::vector<double>> perception_oru::histogram_map::getSimilarityMap(double X,
                                                                                 double Y,
                                                                                 int& indX,
                                                                                 int& indY)
{
    std::vector<std::vector<double>> res(data_grid.size(),
                                         std::vector<double>(data_grid[0].size(), -1.0));
    // int indX, indY;
    getIndexForPoint(X, Y, indX, indY);
    for (int i = 0; i < data_grid.size(); ++i)
    {
        for (int j = 0; j < data_grid[i].size(); ++j)
            if (data_grid[i][j] != NULL)
                res[i][j] = data_grid[i][j]->scoreNoChange(*(data_grid[indX][indY]));
    }
    return res;
}

std::vector<std::vector<double>> perception_oru::histogram_map::getHeatMap()
{
    std::vector<std::vector<double>> res(data_grid.size(),
                                         std::vector<double>(data_grid[0].size(), -1.0));
    for (int i = 0; i < data_grid.size(); ++i)
    {
        for (int j = 0; j < data_grid[i].size(); ++j)
            if (data_grid[i][j] != NULL)
                res[i][j] = data_grid[i][j]->cell_score;
    }
    return res;
}

std::vector<perception_oru::histogram_cell*> perception_oru::histogram_map::getScoredCells()
{

    return data;
}
std::vector<std::vector<double>> perception_oru::histogram_map::getOrientationMap()
{
    std::vector<std::vector<double>> res(data_grid.size(),
                                         std::vector<double>(data_grid[0].size(), -1.0));
    for (int i = 0; i < data_grid.size(); ++i)
    {
        for (int j = 0; j < data_grid[i].size(); ++j)
            if (data_grid[i][j] != NULL)
                res[i][j] = data_grid[i][j]->cell_orientation;
    }
    return res;
}

void perception_oru::histogram_map::reset()
{

    // for(size_t i = 0; i < data.size(); i++)
    //  delete data[i];
}

perception_oru::histogram_map::~histogram_map()
{
    // delete map;
    for (size_t i = 0; i < data.size(); i++)
        delete data[i];
}

void perception_oru::histogram_map::confusion_matrix()
{
    std::ofstream myfile;
    myfile.open("/home/tzkr/conf_mat.m");

    myfile << "conf_mat=[\n";
    for (int i = 0; i < data.size(); ++i)
    {
        for (int j = i; j < data.size(); ++j)
        {
            myfile << i << "," << j << "," << data[i]->score(*(data[j])) << ";\n";
        }
    }
    myfile << "];\n";

    myfile.close();
}
