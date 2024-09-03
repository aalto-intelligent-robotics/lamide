#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>
#include <pcl/common/common.h>

#include <string>
#include <climits>

// #include <ndt_map_lamide/oc_tree.h>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/lazy_grid.h>
#include <ndt_map_lamide/cell_vector.h>
#include <ndt_generic_lamide/math_utils.h>
#include <ndt_generic_lamide/labels.h>

#include <cstring>
#include <cstdio>
#include <ndt_generic_lamide/utils.h>

#include <boost/math/distributions/chi_squared.hpp>

namespace perception_oru
{

//  ██████╗██████╗ ██╗   ██╗██████╗
// ██╔════╝██╔══██╗██║   ██║██╔══██╗
// ██║     ██████╔╝██║   ██║██║  ██║
// ██║     ██╔══██╗██║   ██║██║  ██║
// ╚██████╗██║  ██║╚██████╔╝██████╔╝
//  ╚═════╝╚═╝  ╚═╝ ╚═════╝ ╚═════╝

NDTMap::NDTMap()
{
    index_ = NULL;
    guess_size_ = true;
    map_sizex = map_sizey = map_sizez = -1.0;
    centerx = centery = centerz = 0.0;
    is3D = true;
    guess_size_ = true;
    isFirstLoad_ = true;
}
/** default constructor. The SpatialIndex sent as a paramter
 *	is used as a factory every time that loadPointCloud is called.
 *	it can/should be deallocated outside the class after the destruction of the NDTMap
 */
NDTMap::NDTMap(SpatialIndex* idx, bool dealloc)
{

    index_ = idx;
    // this is used to prevent memory de-allocation of the *si
    // si was allocated outside the NDT class and should be deallocated outside
    isFirstLoad_ = !dealloc;
    map_sizex = map_sizey = map_sizez = -1.0;
    centerx = centery = centerz = 0.0;
    is3D = true;
    guess_size_ = true;
}

NDTMap::NDTMap(const NDTMap& other)
{
    if (other.index_ != NULL)
    {
        this->index_ = other.index_->copy();

        isFirstLoad_ = false;
    }
    float sx, sy, sz;
    other.getMapSize(sx, sy, sz);
    map_sizex = sx;
    map_sizey = sy;
    map_sizez = sz;
    is3D = true;
    guess_size_ = false;
}

/**
 * Construct with given centroid and sizes
 * @param cenx, ceny, cenz; (x,y,z) of the center of the map
 * @param sizex, sizey, sizez: The size of the map in each respective direction
 * NOTE: Implementation only for the laze grid
 **/
NDTMap::NDTMap(SpatialIndex* idx,
               float cenx,
               float ceny,
               float cenz,
               float sizex,
               float sizey,
               float sizez,
               bool dealloc)
{
    if (idx == NULL)
    {
        fprintf(stderr, "Idx == NULL - abort()\n");
        exit(1);
    }
    index_ = idx;

    // this is used to prevent memory de-allocation of the *si
    // si was allocated outside the NDT class and should be deallocated outside
    isFirstLoad_ =
        !dealloc; //////////////////////////////////////////////////////////////////////////////false;
                  // Henrik - was false, but why?

    NDTCell* ptCell = new NDTCell();
    index_->setCellType(ptCell);
    delete ptCell;
    index_->setCenter(cenx, ceny, cenz);
    index_->setSize(sizex, sizey, sizez);
    map_sizex = sizex;
    map_sizey = sizey;
    map_sizez = sizez;
    is3D = true;
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "Unfortunately This constructor works only with Lazygrid!\n");
        exit(1);
    }
    lz->initialize();
    guess_size_ = false;
}

/**
 * Initilize with known values - normally this is done automatically, but in some cases you want
 * to influence these - call only once and before calling any other function
 */
void NDTMap::initialize(double cenx,
                        double ceny,
                        double cenz,
                        double sizex,
                        double sizey,
                        double sizez,
                        double positive_update_static,
                        double negative_update_static,
                        double eta_static,
                        double positive_update_dynamic,
                        double negative_update_dynamic,
                        double eta_dynamic,
                        double w_own,
                        double w_cluster)
{
    isFirstLoad_ = false;

    //   std::cout << " EIGEN " << EIGEN_DEFAULT_ALIGN_BYTES << " " <<
    //   EIGEN_MALLOC_ALREADY_ALIGNED << std::endl;

    //    exit(0);
    NDTCell* ptCell = new NDTCell();
    index_->setCellType(ptCell);
    delete ptCell;
    index_->setCenter(cenx, ceny, cenz);
    index_->setSize(sizex, sizey, sizez);
    map_sizex = sizex;
    map_sizey = sizey;
    map_sizez = sizez;
    is3D = true;
    positive_update_static_ = positive_update_static;
    negative_update_static_ = negative_update_static;
    eta_static_ = eta_static;
    positive_update_dynamic_ = positive_update_dynamic;
    negative_update_dynamic_ = negative_update_dynamic;
    eta_dynamic_ = eta_dynamic;
    w_own_ = w_own;
    w_cluster_ = w_cluster;
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "Unfortunately This constructor works only with Lazygrid!\n");
        exit(1);
    }
    //        lz->initializeAll();
    lz->initialize();
    guess_size_ = false;
}

/**
 * Default destructor
 */
NDTMap::~NDTMap()
{
    // 		std::cout << "NDTMAP destructor index : " << index_ << " is !firstLoad" <<
    // !isFirstLoad_ << std::endl;
    if (index_ != NULL && !isFirstLoad_)
    {
        // 			std::cout << "Destory" << std::endl;
        delete index_;
        index_ = NULL;
    }
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

//  ██████╗ ███████╗████████╗ ██╗███████╗███████╗████████╗
// ██╔════╝ ██╔════╝╚══██╔══╝██╔╝██╔════╝██╔════╝╚══██╔══╝
// ██║  ███╗█████╗     ██║  ██╔╝ ███████╗█████╗     ██║
// ██║   ██║██╔══╝     ██║ ██╔╝  ╚════██║██╔══╝     ██║
// ╚██████╔╝███████╗   ██║██╔╝   ███████║███████╗   ██║
//  ╚═════╝ ╚══════╝   ╚═╝╚═╝    ╚══════╝╚══════╝   ╚═╝

void NDTMap::setParameters(double positive_update_static,
                           double negative_update_static,
                           double eta_static,
                           double positive_update_dynamic,
                           double negative_update_dynamic,
                           double eta_dynamic,
                           double w_own,
                           double w_cluster)
{
    positive_update_static_ = positive_update_static;
    negative_update_static_ = negative_update_static;
    eta_static_ = eta_static;
    positive_update_dynamic_ = positive_update_dynamic;
    negative_update_dynamic_ = negative_update_dynamic;
    eta_dynamic_ = eta_dynamic;
    w_own_ = w_own;
    w_cluster_ = w_cluster;
}

void NDTMap::setMode(bool is3D_)
{
    is3D = is3D_;
}

// Get the cell for which the point fall into (not the closest cell)
bool NDTMap::getCellAtPoint(const pcl::PointXYZL& refPoint, NDTCell*& cell)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    lz->getNDTCellAt(refPoint, cell);
    return (cell != NULL);
}

bool NDTMap::getCellAtPoint(const pcl::PointXYZL& refPoint, NDTCell*& cell) const
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    lz->getNDTCellAt(refPoint, cell);
    return (cell != NULL);
}

// Get the cell for which the point fall into (not the closest cell)
bool NDTMap::getCellAtAllocate(const pcl::PointXYZL& refPoint, NDTCell*& cell)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    lz->getCellAtAllocate(refPoint, cell);
    return (cell != NULL);
}

bool NDTMap::getCellAtAllocate(const pcl::PointXYZL& refPoint, NDTCell*& cell) const
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    lz->getCellAtAllocate(refPoint, cell);
    return (cell != NULL);
}

/**
 * Set the map size in meters - Must be called before first Cloud call if
 * you want to set the size - otherwise it is automatically determined
 */
void NDTMap::setMapSize(float sx, float sy, float sz)
{
    map_sizex = sx;
    map_sizey = sy;
    map_sizez = sz;
}

void NDTMap::getMapSize(float& sx, float& sy, float& sz) const
{
    sx = map_sizex;
    sy = map_sizey;
    sz = map_sizez;
}

bool NDTMap::getCentroid(double& cx, double& cy, double& cz) const
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getCenter(cx, cy, cz);
    return true;
}

bool NDTMap::setCentroid(double cx, double cy, double cz)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->setCenter(cx, cy, cz);
    return true;
}

bool NDTMap::getGridSize(int& cx, int& cy, int& cz)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getGridSize(cx, cy, cz);
    return true;
}

bool NDTMap::getGridSizeInMeters(double& cx, double& cy, double& cz)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getGridSizeInMeters(cx, cy, cz);
    return true;
}

bool NDTMap::getGridSizeInMeters(double& cx, double& cy, double& cz) const
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getGridSizeInMeters(cx, cy, cz);
    return true;
}

bool NDTMap::getCellSizeInMeters(double& cx, double& cy, double& cz)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getCellSize(cx, cy, cz);
    return true;
}

bool NDTMap::getCellSizeInMeters(double& cx, double& cy, double& cz) const
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getCellSize(cx, cy, cz);
    return true;
}

double NDTMap::getSmallestCellSizeInMeters() const
{
    double cx, cy, cz;
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
        return false;
    lz->getCellSize(cx, cy, cz);

    return std::min(std::min(cx, cy), cz);
}

// Get estimated depth
// This goes through now the cells always to max depth and is not the most efficient solution
double NDTMap::getDepth(Eigen::Vector3d origin, Eigen::Vector3d dir, double maxDepth)
{
    // std::cout << "from: ";
    // std::cout << origin.x() << " ";
    // std::cout << origin.y() << " ";
    // std::cout << origin.z() << " ";
    // std::cout << std::endl;
    // std::cout << "dir: ";
    // std::cout << dir.x() << " ";
    // std::cout << dir.y() << " ";
    // std::cout << dir.z() << " ";
    // std::cout << std::endl;

    Eigen::Vector3d ray_endpos = origin + dir * maxDepth;
    std::vector<NDTCell*> cells;

    Eigen::Vector3d diff = ray_endpos - origin;
    pcl::PointXYZL endP;
    endP.x = ray_endpos(0);
    endP.y = ray_endpos(1);
    endP.z = ray_endpos(2);

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }

    if (!lz->traceLine(origin, endP, diff, 1000.0, cells))
    {
        return -1;
    }
    // fprintf(stderr,"Got trace with %d Cells (%lf)\n",cells.size(),(ray_endpos-origin).norm());
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);

    Eigen::Vector3d out;
    bool hasML = false;

    for (unsigned int i = 0; i < cells.size(); i++)
    {
        if (cells[i]->hasGaussian_)
        {
            double lik = cells[i]->computeMaximumLikelihoodAlongLine(po, endP, out);
            if (lik > 0.1)
            {
                // fprintf(stderr,"Got ML %lf (%lf)\n",lik,(out-origin).norm());
                hasML = true;
                break;
            }
            else
            {
                // std::cout << "too unlikely" << std::endl;
            }
        }
        else
        {
            // std::cout << "no gaussian" << std::endl;
        }
    }

    if (hasML)
    {
        return (out - origin).norm();
    }

    return -1;
}

std::pair<double, int> NDTMap::getDepthAndLabel(Eigen::Vector3d origin,
                                                Eigen::Vector3d dir,
                                                double maxDepth)
{
    // std::cout << "from: ";
    // std::cout << origin.x() << " ";
    // std::cout << origin.y() << " ";
    // std::cout << origin.z() << " ";
    // std::cout << std::endl;
    // std::cout << "dir: ";
    // std::cout << dir.x() << " ";
    // std::cout << dir.y() << " ";
    // std::cout << dir.z() << " ";
    // std::cout << std::endl;
    int label = -1;
    Eigen::Vector3d ray_endpos = origin + dir * maxDepth;
    std::vector<NDTCell*> cells;

    Eigen::Vector3d diff = ray_endpos - origin;
    pcl::PointXYZL endP;
    endP.x = ray_endpos(0);
    endP.y = ray_endpos(1);
    endP.z = ray_endpos(2);

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }

    if (!lz->traceLine(origin, endP, diff, 1000.0, cells))
    {
        return std::make_pair(-1.0, label);
    }
    // fprintf(stderr,"Got trace with %d Cells (%lf)\n",cells.size(),(ray_endpos-origin).norm());
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);

    Eigen::Vector3d out;
    bool hasML = false;

    for (unsigned int i = 0; i < cells.size(); i++)
    {
        if (cells[i]->hasGaussian_)
        {
            double lik = cells[i]->computeMaximumLikelihoodAlongLine(po, endP, out);
            if (lik > 0.1)
            {
                // fprintf(stderr,"Got ML %lf (%lf)\n",lik,(out-origin).norm());
                label = cells[i]->getLabel();
                hasML = true;
                break;
            }
            else
            {
                // std::cout << "too unlikely" << std::endl;
            }
        }
        else
        {
            // std::cout << "no gaussian" << std::endl;
        }
    }

    if (hasML)
    {
        return std::make_pair((out - origin).norm(), label);
    }

    return std::make_pair(-1.0, label);
}

double NDTMap::getDepthSmooth(Eigen::Vector3d origin,
                              Eigen::Vector3d dir,
                              double maxDepth,
                              int n_neigh,
                              double weight,
                              double threshold,
                              Eigen::Vector3d* hit)
{
    Eigen::Vector3d ray_endpos = origin + dir * maxDepth;
    std::vector<NDTCell*> cells, surr;

    Eigen::Vector3d diff = ray_endpos - origin;
    pcl::PointXYZL endP;
    endP.x = ray_endpos(0);
    endP.y = ray_endpos(1);
    endP.z = ray_endpos(2);

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }

    if (!lz->traceLine(origin, endP, diff, 1000.0, cells))
    {
        return maxDepth + 1.0;
    }

    pcl::PointXYZL startP, p;
    startP.x = origin(0);
    startP.y = origin(1);
    startP.z = origin(2);

    Eigen::Vector3d out;
    bool hasML = false;

    for (unsigned int i = 0; i < cells.size(); i++)
    {
        if (cells[i]->hasGaussian_)
        {
            surr = lz->getClosestNDTCells(cells[i]->getCenter(), n_neigh, true);
            double like = cells[i]->computeMaximumLikelihoodAlongLine(startP, endP, out);
            p.x = out(0);
            p.y = out(1);
            p.z = out(2);
            for (unsigned int k = 1u; k < surr.size(); ++k)
            {
                like += weight * surr[k]->getLikelihood(p);
            }
            if (like > threshold)
            {
                hasML = true;
                break;
            }
        }
    }

    if (hasML)
    {
        if (hit != NULL)
        {
            *hit = out;
        }
        return (out - origin).norm();
    }

    return (maxDepth + 1.0);
}

// returns the current spatial index as an integer (debugging function)
int NDTMap::getMyIndexInt() const
{
    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        return 1;
    }

    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    if (gr != NULL)
    {
        return 3;
    }

    return -1;
}

// computes the *negative log likelihood* of a single observation
double NDTMap::getLikelihoodForPoint(pcl::PointXYZL pt)
{
    double uniform = 0.00100;
    NDTCell* ndCell = NULL;

    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    if (gr == NULL)
    {
        return uniform;
    }
    ndCell = gr->getClosestNDTCell(pt);

    if (ndCell == NULL)
    {
        return uniform;
    }

    double prob = ndCell->getLikelihood(pt);
    // uniform!! TSV
    prob = (prob < 0) ? 0 : prob;
    return prob;
}

std::vector<NDTCell*> NDTMap::getInitializedCellsForPoint(const pcl::PointXYZL pt) const
{
    std::vector<NDTCell*> cells;
    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    if (gr == NULL)
    {
        return cells;
    }
    cells = gr->getClosestCells(pt);
    return cells;
}

std::vector<NDTCell*> NDTMap::getCellsForPoint(const pcl::PointXYZL pt,
                                               int n_neigh,
                                               bool checkForGaussian) const
{
    std::vector<NDTCell*> cells;
    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    if (gr == NULL)
    {
        return cells;
    }
    cells = gr->getClosestNDTCells(pt, n_neigh, checkForGaussian);
    return cells;
}

bool NDTMap::getCellForPoint(const pcl::PointXYZL& pt,
                             NDTCell*& out_cell,
                             bool checkForGaussian) const
{

    out_cell = NULL;
    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        out_cell = cl->getClosestNDTCell(pt);
        return true;
    }

    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    if (gr != NULL)
    {
        out_cell = gr->getClosestNDTCell(pt, checkForGaussian);
        return true;
    }
    // cout<<"bad index - getCellForPoint\n";
    return false;
}

unsigned int NDTMap::getOverlap(pcl::PointCloud<pcl::PointXYZL>& cloud)
{
    unsigned int nr_overlap = 0;
    for (int i = 0; i < cloud.size(); i++)
    {
        pcl::PointXYZL pt = cloud[i];
        std::vector<NDTCell*> overlapping_cells = getCellsForPoint(pt, 1, true);
        for (int i = 0; i < overlapping_cells.size(); i++)
        {
            if (overlapping_cells[i]->hasGaussian_)
            {
                nr_overlap++;
                break;
            }
        }
    }
    return nr_overlap;
}

// returns the current spatial index as a string (debugging function)
std::string NDTMap::getMyIndexStr() const
{
    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        return std::string("CellVector");
    }

    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    if (gr != NULL)
    {
        return std::string("LazyGrid<PointT>");
    }

    return std::string("Unknown index type");
}

NDTCell* NDTMap::getCellIdx(unsigned int idx) const
{
    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        return cl->getCellIdx(idx);
    }
    return NULL;
}

std::vector<NDTCell*> NDTMap::getAllCells() const
{

    std::vector<NDTCell*> ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell->hasGaussian_)
        {
            //	    NDTCell* nd = cell->copy();
            ret.push_back(cell);
        }
        it++;
    }
    return ret;
}

std::vector<NDTCell*> NDTMap::getAllCellsNoCopy() const
{

    std::vector<NDTCell*> ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell->hasGaussian_)
        {
            ret.push_back(cell);
        }
        it++;
    }
    return ret;
}

std::vector<boost::shared_ptr<NDTCell>> NDTMap::getAllCellsShared() const
{

    std::vector<boost::shared_ptr<NDTCell>> ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {

        NDTCell* cell = (*it);
        if (cell->hasGaussian_)
        {

            NDTCell* nd = cell->copy();
            boost::shared_ptr<NDTCell> smart_pointer(nd);
            // 			NDTCell** ndd = &nd;
            ret.push_back(smart_pointer);
        }
        it++;
    }
    //    std::cout << "Return " << ret.size() << std::endl;
    return ret;
}

std::vector<NDTCell*> NDTMap::getAllInitializedCells() const
{
    std::vector<NDTCell*> ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        // REMOVED THE COPY SINCE:
        //  COPY SHOULD BE SMART POINTERS
        //  IF POINTERS ARE RETURNED THEN IT SHOULD NOT HAVE BEEN COPIED AND WE RETURN THE ACTUAL
        //  DATATYPE
        //	NDTCell* nd = (*it)->copy();
        ret.push_back(*it);

        it++;
    }
    return ret;
}

std::vector<boost::shared_ptr<NDTCell>> NDTMap::getAllInitializedCellsShared() const
{
    std::vector<boost::shared_ptr<NDTCell>> ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        NDTCell* nd = (*it)->copy();
        boost::shared_ptr<NDTCell> smart_pointer(nd);
        ret.push_back(smart_pointer);
        it++;
    }
    return ret;
}

NDTCell* NDTMap::getCellAtID(int x, int y, int z) const
{
    NDTCell* cell;
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    lz->getCellAt(x, y, z, cell);
    return cell;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

//  █████╗ ██████╗ ██████╗      ██████╗██╗      ██████╗ ██╗   ██╗██████╗
// ██╔══██╗██╔══██╗██╔══██╗    ██╔════╝██║     ██╔═══██╗██║   ██║██╔══██╗
// ███████║██║  ██║██║  ██║    ██║     ██║     ██║   ██║██║   ██║██║  ██║
// ██╔══██║██║  ██║██║  ██║    ██║     ██║     ██║   ██║██║   ██║██║  ██║
// ██║  ██║██████╔╝██████╔╝    ╚██████╗███████╗╚██████╔╝╚██████╔╝██████╔╝
// ╚═╝  ╚═╝╚═════╝ ╚═════╝      ╚═════╝╚══════╝ ╚═════╝  ╚═════╝ ╚═════╝

/**
 * Adds a new cloud: NDT-OM update step
 */
void NDTMap::addPointCloud(const Eigen::Vector3d& origin,
                           const pcl::PointCloud<pcl::PointXYZL>& pc,
                           double classifierTh,
                           double maxz,
                           double sensor_noise,
                           double occupancy_limit)
{
    // 	std::cout << "Good function: addPointCloud" << std::endl;
    // 	exit(0);
    if (isFirstLoad_)
    {
        loadPointCloud(pc);
        return;
    }
    if (index_ == NULL)
    {
        // ERR("Problem creating index, unimplemented method\n");
        return;
    }
    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = pc.points.begin();

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    pcl::PointXYZL po, pt;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);
    NDTCell* ptCell = NULL;

    std::vector<NDTCell*> cells;
    bool updatePositive = true;
    double max_range = 200.;

    while (it != pc.points.end())
    {
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }

        Eigen::Vector3d diff;
        diff << it->x - origin(0), it->y - origin(1), it->z - origin(2);
        double l = diff.norm();

        if (l > max_range)
        {
            fprintf(stderr, "addPointCloud::Very long distance (%lf) :( \n", l);
            it++;
            continue;
        }

        cells.clear();
        // 			std::cout << "Tracing the line" << std::endl;
        if (!lz->traceLine(origin, *it, diff, maxz, cells))
        {
            it++;
            continue;
        }

        for (unsigned int i = 0; i < cells.size(); i++)
        {
            ptCell = cells[i];
            if (ptCell != NULL)
            {
                double l2target = 0;
                if (ptCell->hasGaussian_)
                {
                    Eigen::Vector3d out, pend, vpt;
                    pend << it->x, it->y, it->z;
                    double lik = ptCell->computeMaximumLikelihoodAlongLine(po, *it, out);
                    l2target = (out - pend).norm();

                    double dist = (origin - out).norm();
                    if (dist > l)
                    {
                        continue; ///< don't accept points further than the measurement
                    }

                    l2target = (out - pend).norm(); ///< distance to endpoint

                    double sigma_dist = 0.5 * (dist / 30.0); // test for distance based sensor
                                                             // noise
                    double snoise = sigma_dist + sensor_noise;
                    // This is the probability of max lik point being endpoint
                    double thr = exp(-0.5 * (l2target * l2target) / (snoise * snoise));
                    lik *= (1.0 - thr);
                    if (lik < 0.3)
                    {
                        continue;
                    }
                    lik = -0.1 * lik + 0.5; // Evidence value for empty - alpha * p(x);
                    double logoddlik = logOdds(lik);
                    // ptCell->updateEmpty(logoddlik,l2target);
                    // fprintf(stderr,"[E=%.2lf] ", logoddlik);
                    ptCell->updateOccupancy(logoddlik, occupancy_limit);
                    if (ptCell->getOccupancy() <= 0)
                    {
                        ptCell->hasGaussian_ = false;
                    }
                }
                else
                {
                    // ptCell->updateEmpty(-0.2,l2target); ///The cell does not have gaussian, so we
                    // mark that we saw it empty...
                    ptCell->updateOccupancy(-0.2, occupancy_limit);
                    if (ptCell->getOccupancy() <= 0)
                    {
                        ptCell->hasGaussian_ = false;
                    }
                }
            }
        }
        if (updatePositive)
        {
            ptCell = dynamic_cast<NDTCell*>(index_->addPoint(*it));
            if (ptCell != NULL)
            {
                update_set.insert(ptCell);
            }
        }
        it++;
    }
    isFirstLoad_ = false;
}

// ███████╗██╗███╗   ███╗██████╗ ██╗     ███████╗
// ██╔════╝██║████╗ ████║██╔══██╗██║     ██╔════╝
// ███████╗██║██╔████╔██║██████╔╝██║     █████╗
// ╚════██║██║██║╚██╔╝██║██╔═══╝ ██║     ██╔══╝
// ███████║██║██║ ╚═╝ ██║██║     ███████╗███████╗
// ╚══════╝╚═╝╚═╝     ╚═╝╚═╝     ╚══════╝╚══════╝

/**
 * Just adds the points, without raytracing and such
 */
void NDTMap::addPointCloudSimple(const pcl::PointCloud<pcl::PointXYZL>& pc, double maxz)
{
    if (isFirstLoad_)
    {
        loadPointCloud(pc);
        return;
    }

    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = pc.points.begin();
    it = pc.points.begin();
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "Unfortunately This works only with Lazygrid!\n");
        exit(1);
    }

    while (it != pc.points.end())
    {
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }
        if (it->z > maxz)
        {
            it++;
            continue;
        }
        index_->addPoint(*it);
        NDTCell* ptCell;
        lz->getNDTCellAt(*it, ptCell);

        if (ptCell != NULL)
        {
            update_set.insert(ptCell);
        }

        it++;
    }
}

// ██████╗ ██╗███████╗████████╗
// ██╔══██╗██║██╔════╝╚══██╔══╝
// ██║  ██║██║███████╗   ██║
// ██║  ██║██║╚════██║   ██║
// ██████╔╝██║███████║   ██║
// ╚═════╝ ╚═╝╚══════╝   ╚═╝

/**
 * Add a distribution to the map
 */
void NDTMap::addDistributionToCell(const Eigen::Matrix3d& ucov,
                                   const Eigen::Vector3d& umean,
                                   unsigned int numpointsindistribution,
                                   unsigned int maxnumpoints,
                                   float max_occupancy,
                                   int label,
                                   int weight)
{
    pcl::PointXYZL pt;
    pt.x = umean[0];
    pt.y = umean[1];
    pt.z = umean[2];
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    NDTCell* ptCell = NULL;
    lz->getCellAtAllocate(pt, ptCell);

    if (ptCell != NULL)
    {
        double positiveOccupancy = getPositiveUpdate(ptCell);
        ptCell->updateSampleVariance(ucov, umean, numpointsindistribution, true, positiveOccupancy,
                                     max_occupancy, maxnumpoints);

        // FIXME: set the label
        ptCell->setLabel(label, weight);

        // debug
        ptCell->viz_updated_ = true;
    }
}

// ███╗   ███╗███████╗ █████╗ ███╗   ██╗    ██╗   ██╗██████╗ ██████╗  █████╗ ████████╗███████╗
// ████╗ ████║██╔════╝██╔══██╗████╗  ██║    ██║   ██║██╔══██╗██╔══██╗██╔══██╗╚══██╔══╝██╔════╝
// ██╔████╔██║█████╗  ███████║██╔██╗ ██║    ██║   ██║██████╔╝██║  ██║███████║   ██║   █████╗
// ██║╚██╔╝██║██╔══╝  ██╔══██║██║╚██╗██║    ██║   ██║██╔═══╝ ██║  ██║██╔══██║   ██║   ██╔══╝
// ██║ ╚═╝ ██║███████╗██║  ██║██║ ╚████║    ╚██████╔╝██║     ██████╔╝██║  ██║   ██║   ███████╗
// ╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝     ╚═════╝ ╚═╝     ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝

/**
 * Add new pointcloud to map - Updates the occupancy using the mean values of
 * a local map generated from an observation
 *
 * Performs raytracing, updates conflicts and adds points to cells
 * computeNDTCells must be called after calling this
 *
 * @param &origin is the position of the sensor, from where the scan has been taken from.
 * @param &pc is the pointcloud to be added
 * @param &localmapsize The dimensions of the local map used for computing the gaussians
 * @param maxnumpoints Defines the forgetting factor (default 100000) the smaller the value the
 * faster the adaptation
 * @param occupancy_limit Clamping threshold for log-odds value
 * @param maxz threshold for the maximum z-coordinate value for the measurement point_cloud
 * @param sensor_noise The expected standard deviation of the sensor noise
 */
void NDTMap::addPointCloudMeanUpdate(const Eigen::Vector3d& origin,
                                     const pcl::PointCloud<pcl::PointXYZL>& pc,
                                     const Eigen::Vector3d& localmapsize,
                                     unsigned int maxnumpoints,
                                     float occupancy_limit,
                                     double maxz,
                                     double sensor_noise)
{
    ndt_generic::Stopwatch s("standard update");

    if (isFirstLoad_)
    {
        loadPointCloud(pc);
        computeNDTCells();
        return;
    }
    if (index_ == NULL)
    {
        return;
    }

    //--------------------------------------------------------------------------
    // Create update map
    //--------------------------------------------------------------------------

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }

    double centerX, centerY, centerZ;
    lz->getCenter(centerX, centerY, centerZ);
    double cellSizeX, cellSizeY, cellSizeZ;
    lz->getCellSize(cellSizeX, cellSizeY, cellSizeZ);
    int sizeX, sizeY, sizeZ;
    lz->getGridSize(sizeX, sizeY, sizeZ);

    Eigen::Vector3d old_centroid;
    old_centroid(0) = centerX;
    old_centroid(1) = centerY;
    old_centroid(2) = centerZ;

    double min1 = std::min(cellSizeX, cellSizeY);
    double min2 = std::min(cellSizeZ, cellSizeY);

    // Select the smallest resolution
    double resolution = std::min(min1, min2);

    // Lets first create a local ndmap (this really works only if we have lazy grid as well)
    perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution));
    // No range limit used here, assume that the point cloud is already filtered.
    ndlocal.loadPointCloudCentroid(pc, origin, old_centroid, localmapsize, -1.);

    // Use Student-T
    // ndlocal.computeNDTCells(CELL_UPDATE_MODE_STUDENT_T);

    // Use Sample variance
    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    std::vector<NDTCell*> ndts;
    ndts = ndlocal.getAllCells();
    NDTCell* ptCell = NULL;

    pcl::PointXYZL pt;
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);
    int num_high = 0;

    // debug
    double max_p = -1;
    double max_logodd = -1e5;
    double max_pt = -1;
    double max_up = -1e7;
    double min_p = 2;
    double min_logodd = 1e5;
    double min_pt = 1e7;
    double min_up = 1e7;

    s.lap("init");

    for (unsigned int it = 0; it < ndts.size(); it++)
    {
        if (ndts[it] == NULL)
        {
            fprintf(
                stderr,
                "NDTMap::addPointCloudMeanUpdate::GOT NULL FROM MAP -- SHOULD NEVER HAPPEN!!!\n");
            continue;
        }

        if (!ndts[it]->hasGaussian_)
        {
            fprintf(stderr,
                    "NDTMap::addPointCloudMeanUpdate::NO GAUSSIAN!!!! -- SHOULD NEVER HAPPEN!!!\n");
            continue;
        }

        int numpoints = ndts[it]->getN();

        if (numpoints <= 0)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::Number of points in distribution<=0!!");
            continue;
        }
        Eigen::Vector3d mean = ndts[it]->getMean();
        Eigen::Vector3d diff = mean - origin;

        // Neareast neighbor search
        double raylen = diff.norm();
        int NN = raylen / (resolution);

        if (raylen > 200)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::Very long distance (%lf) :( \n", raylen);
            continue;
        }
        if (resolution < 0.01)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::Resolution very very small (%lf) :( \n",
                    resolution);
            continue;
        }
        if (NN < 0)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::N=%d (r=%lf l=%lf) :( ", NN, resolution,
                    raylen);
            continue;
        }

        bool withinHeightLimit = mean(2) <= maxz;
        if (!withinHeightLimit)
        {
            // Lets update negative even though the measurement was too high
            num_high++;
        }

        //----------------------------------------------------------------------
        // Trace line
        //----------------------------------------------------------------------

        diff = diff / (float)NN;
        int idxo = 0, idyo = 0, idzo = 0;
        for (int i = 0; i < NN - 2; i++)
        {
            pt.x = origin(0) + ((float)(i + 1)) * diff(0);
            pt.y = origin(1) + ((float)(i + 1)) * diff(1);
            pt.z = origin(2) + ((float)(i + 1)) * diff(2);
            int idx, idy, idz;

            idx = (int)(((pt.x - centerX) / cellSizeX + 0.5) + sizeX / 2.0);
            idy = (int)(((pt.y - centerY) / cellSizeY + 0.5) + sizeY / 2.0);
            idz = (int)(((pt.z - centerZ) / cellSizeZ + 0.5) + sizeZ / 2.0);

            // We only want to check every cell once, so
            // increase the index if we are still in the same cell
            if (idx == idxo && idy == idyo && idz == idzo)
            {
                continue;
            }
            else
            {
                idxo = idx;
                idyo = idy;
                idzo = idz;
            }

            //------------------------------------------------------------------
            // Get cell
            //------------------------------------------------------------------

            ptCell = NULL;
            // Check the validity of the index
            lz->getNDTCellAt(idx, idy, idz, ptCell);
            if (ptCell == NULL)
            {
                // Add fake point to initialize!
                ptCell = dynamic_cast<NDTCell*>(index_->addPoint(pt));
                continue;
            }

            //------------------------------------------------------------------
            // Empty update
            //------------------------------------------------------------------

            if (!ptCell->hasGaussian_)
            {
                // The cell does not have gaussian,
                // so we mark that we saw it empty...
                double negative_update = getNegativeUpdate(ptCell);
                double lneg = logOdds(negative_update);
                float update = lneg * numpoints;
                ptCell->updateOccupancy(update, occupancy_limit);
                continue;
            }

            //------------------------------------------------------------------
            // Gaussian comparison
            //------------------------------------------------------------------

            Eigen::Vector3d ML_point;
            Eigen::Vector3d vpt;

            // likelihood of x_ML (maximum likelihood) given N(r,sigma)
            double maxLikelihood = ptCell->computeMaximumLikelihoodAlongLine(po, pt, ML_point);

            // test for distance based sensor noise
            // calculate sigma
            double dist = (origin - ML_point).norm();

            // don't accept points further than the measurement
            if (dist > raylen)
            {
                continue;
            }

            // FIXME: constants from original codebase!
            double sigma_dist = 0.5 * (dist / 30.0);
            double snoise = sigma_dist + sensor_noise;

            // distance to endpoint
            double l2target = (ML_point - mean).norm();

            // This is the probability of ML point being endpoint
            // likelihood of p_xML given z
            double hit_probability = exp(-0.5 * (l2target * l2target) / (snoise * snoise));

            // Evidence value for empty - eta * p(x);
            double eta = getEta(ptCell);
            double p_occupancy = 0.5 - eta * (maxLikelihood * (1.0 - hit_probability));
            p_occupancy = clamp(p_occupancy, 0.0, 1.0);
            double logOddsLikelihood = logOdds(p_occupancy);

            // note: original formulas for reference. log odds was reversed!
            // double p_occupancy = 0.5 + 0.2 * (maxLikelihood * (1.0 - hit_probability));
            // double logOddsLikelihood = log((1.0 - p_occupancy) / (p_occupancy));

            // debug
            if (p_occupancy > max_p)
            {
                max_p = p_occupancy;
            }
            if (logOddsLikelihood > max_logodd)
            {
                max_logodd = logOddsLikelihood;
            }
            if (numpoints > max_pt)
            {
                max_pt = numpoints;
            }
            if (numpoints * logOddsLikelihood > max_up)
            {
                max_up = numpoints * logOddsLikelihood;
            }
            if (p_occupancy < min_p)
            {
                min_p = p_occupancy;
            }
            if (logOddsLikelihood < min_logodd)
            {
                min_logodd = logOddsLikelihood;
            }
            if (numpoints < min_pt)
            {
                min_pt = numpoints;
            }
            if (numpoints * logOddsLikelihood < min_up)
            {
                min_up = numpoints * logOddsLikelihood;
            }

            // debug
            ptCell->viz_accessed_ = true;

            // update
            float update = numpoints * logOddsLikelihood;
            ptCell->updateOccupancy(update, occupancy_limit);
            if (ptCell->getOccupancy() < 0)
            {
                ptCell->hasGaussian_ = false;
            }
        }

        //------------------------------------------------------------------
        // Positive update ("hit")
        //------------------------------------------------------------------
        if (withinHeightLimit)
        {
            Eigen::Matrix3d ucov = ndts[it]->getCov();
            int label = ndts[it]->getLabel();
            int weight = ndts[it]->getLabelWeight();
            // ORU_FIXME:: local implementation can be faster?
            // this updates occupancy as well
            addDistributionToCell(ucov, mean, numpoints, maxnumpoints, occupancy_limit, label,
                                  weight);
        }
    }

    s.lap("update");
    s.stop();
    s.print();

    isFirstLoad_ = false;

    // DEBUG
    // std::cout << "Update done" << std::endl;
    // std::cout << "max_p:      " << max_p << std::endl;
    // std::cout << "min_p:      " << min_p << std::endl;
    // std::cout << "max_logodd: " << max_logodd << std::endl;
    // std::cout << "min_logodd: " << min_logodd << std::endl;
    // std::cout << "max_pt:     " << max_pt << std::endl;
    // std::cout << "min_pt:     " << min_pt << std::endl;
    // std::cout << "max_up:     " << max_up << std::endl;
    // std::cout << "min_up:     " << min_up << std::endl;
}

// original was -0.85
double NDTMap::getNegativeUpdate(const NDTCell* cell)
{
    int label = cell->getLabel();
    bool dynamic = !ndt_generic::isStatic(label);
    return dynamic ? negative_update_dynamic_ : negative_update_static_;
}

// original was 0.6
double NDTMap::getPositiveUpdate(const NDTCell* cell)
{
    int label = cell->getLabel();
    bool dynamic = !ndt_generic::isStatic(label);
    return dynamic ? positive_update_dynamic_ : positive_update_static_;
}

// original was 0.2
double NDTMap::getEta(const NDTCell* cell)
{
    int label = cell->getLabel();
    bool dynamic = !ndt_generic::isStatic(label);
    return dynamic ? eta_dynamic_ : eta_static_;
}

//  ██████╗██╗     ██╗   ██╗███████╗████████╗███████╗██████╗
// ██╔════╝██║     ██║   ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗
// ██║     ██║     ██║   ██║███████╗   ██║   █████╗  ██████╔╝
// ██║     ██║     ██║   ██║╚════██║   ██║   ██╔══╝  ██╔══██╗
// ╚██████╗███████╗╚██████╔╝███████║   ██║   ███████╗██║  ██║
//  ╚═════╝╚══════╝ ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝

// ██╗   ██╗██████╗ ██████╗  █████╗ ████████╗███████╗
// ██║   ██║██╔══██╗██╔══██╗██╔══██╗╚══██╔══╝██╔════╝
// ██║   ██║██████╔╝██║  ██║███████║   ██║   █████╗
// ██║   ██║██╔═══╝ ██║  ██║██╔══██║   ██║   ██╔══╝
// ╚██████╔╝██║     ██████╔╝██║  ██║   ██║   ███████╗
//  ╚═════╝ ╚═╝     ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝

void NDTMap::addPointCloudClusterUpdate(const Eigen::Vector3d& origin,
                                        const pcl::PointCloud<pcl::PointXYZL>& pc,
                                        const Eigen::Vector3d& localmapsize,
                                        unsigned int maxnumpoints,
                                        float occupancy_limit,
                                        double maxz,
                                        double sensor_noise)
{
    if (isFirstLoad_)
    {
        loadPointCloud(pc);
        computeNDTCells();
        cluster(true, false, false);
        return;
    }
    if (index_ == NULL)
    {
        return;
    }

    //--------------------------------------------------------------------------
    // Create update map
    //--------------------------------------------------------------------------

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }

    double centerX, centerY, centerZ;
    lz->getCenter(centerX, centerY, centerZ);
    double cellSizeX, cellSizeY, cellSizeZ;
    lz->getCellSize(cellSizeX, cellSizeY, cellSizeZ);
    int sizeX, sizeY, sizeZ;
    lz->getGridSize(sizeX, sizeY, sizeZ);

    Eigen::Vector3d old_centroid;
    old_centroid(0) = centerX;
    old_centroid(1) = centerY;
    old_centroid(2) = centerZ;

    double min1 = std::min(cellSizeX, cellSizeY);
    double min2 = std::min(cellSizeZ, cellSizeY);

    // Select the smallest resolution
    double resolution = std::min(min1, min2);

    // Lets first create a local ndmap (this really works only if we have lazy grid as well)
    perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution));
    // No range limit used here, assume that the point cloud is already filtered.
    ndlocal.loadPointCloudCentroid(pc, origin, old_centroid, localmapsize, -1.);

    // Use Sample variance
    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    std::vector<NDTCell*> ndts;
    ndts = ndlocal.getAllCells();
    NDTCell* ptCell = NULL;

    pcl::PointXYZL pt;
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);
    int num_high = 0;

    // udpate container
    int maxCluster = getMaxClusterId();
    int updateUpperBound = ndts.size() * (200.0 / resolution);
    EvidenceContainer evidence;

    for (unsigned int it = 0; it < ndts.size(); it++)
    {
        if (ndts[it] == NULL)
        {
            fprintf(
                stderr,
                "NDTMap::addPointCloudMeanUpdate::GOT NULL FROM MAP -- SHOULD NEVER HAPPEN!!!\n");
            continue;
        }

        if (!ndts[it]->hasGaussian_)
        {
            fprintf(stderr,
                    "NDTMap::addPointCloudMeanUpdate::NO GAUSSIAN!!!! -- SHOULD NEVER HAPPEN!!!\n");
            continue;
        }

        int numpoints = ndts[it]->getN();

        if (numpoints <= 0)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::Number of points in distribution<=0!!");
            continue;
        }
        Eigen::Vector3d mean = ndts[it]->getMean();
        Eigen::Vector3d diff = mean - origin;

        // Neareast neighbor search
        double raylen = diff.norm();
        int NN = raylen / (resolution);

        if (raylen > 200)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::Very long distance (%lf) :( \n", raylen);
            continue;
        }
        if (resolution < 0.01)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::Resolution very very small (%lf) :( \n",
                    resolution);
            continue;
        }
        if (NN < 0)
        {
            fprintf(stderr, "addPointCloudMeanUpdate::N=%d (r=%lf l=%lf) :( ", NN, resolution,
                    raylen);
            continue;
        }

        bool withinHeightLimit = mean(2) <= maxz;
        if (!withinHeightLimit)
        {
            // Lets update negative even though the measurement was too high
            num_high++;
        }

        //----------------------------------------------------------------------
        // Trace line
        //----------------------------------------------------------------------

        diff = diff / (float)NN;
        int idxo = 0, idyo = 0, idzo = 0;
        for (int i = 0; i < NN - 2; i++)
        {
            pt.x = origin(0) + ((float)(i + 1)) * diff(0);
            pt.y = origin(1) + ((float)(i + 1)) * diff(1);
            pt.z = origin(2) + ((float)(i + 1)) * diff(2);
            int idx, idy, idz;

            idx = (int)(((pt.x - centerX) / cellSizeX + 0.5) + sizeX / 2.0);
            idy = (int)(((pt.y - centerY) / cellSizeY + 0.5) + sizeY / 2.0);
            idz = (int)(((pt.z - centerZ) / cellSizeZ + 0.5) + sizeZ / 2.0);

            // We only want to check every cell once, so
            // increase the index if we are still in the same cell
            if (idx == idxo && idy == idyo && idz == idzo)
            {
                continue;
            }
            else
            {
                idxo = idx;
                idyo = idy;
                idzo = idz;
            }

            //------------------------------------------------------------------
            // Get cell
            //------------------------------------------------------------------

            ptCell = NULL;
            // Check the validity of the index
            lz->getNDTCellAt(idx, idy, idz, ptCell);
            if (ptCell == NULL)
            {
                // Add fake point to initialize!
                ptCell = dynamic_cast<NDTCell*>(index_->addPoint(pt));
            }
            // If creation fails
            if (ptCell == NULL)
            {
                continue;
            }

            int clusterId = ptCell->getClusterId();

            //------------------------------------------------------------------
            // Empty update
            //------------------------------------------------------------------

            if (!ptCell->hasGaussian_)
            {
                // The cell does not have gaussian,
                // so we mark that we saw it empty...
                double negative_update = getNegativeUpdate(ptCell);
                // evidence.add(ptCell, negative_update, numpoints);
                ptCell->updateOccupancy(logOdds(negative_update) * numpoints, occupancy_limit);
                continue;
            }

            //------------------------------------------------------------------
            // Gaussian comparison
            //------------------------------------------------------------------

            Eigen::Vector3d ML_point;
            Eigen::Vector3d vpt;

            // likelihood of x_ML (maximum likelihood) given N(r,sigma)
            // i.e. how likely ray goes through the distribution
            double hit_probability = ptCell->computeMaximumLikelihoodAlongLine(po, pt, ML_point);

            // test for distance based sensor noise
            // calculate sigma
            double dist = (origin - ML_point).norm();

            // don't accept points further than the measurement
            if (dist > raylen)
            {
                continue;
            }

            // FIXME: constants from original codebase!
            double sigma_dist = 0.5 * (dist / 30.0);
            double snoise = sigma_dist + sensor_noise;

            // distance to endpoint
            double l2target = (ML_point - mean).norm();

            // This is the probability of ML point being endpoint
            // likelihood of p_xML given z
            double end_probability = exp(-0.5 * (l2target * l2target) / (snoise * snoise));

            // Evidence value for empty - eta * p(x);
            double eta = getEta(ptCell);
            double p_occupancy = 0.5 - eta * hit_probability * (1.0 - end_probability);
            double logOddsLikelihood = logOdds(p_occupancy);

            // debug
            ptCell->viz_accessed_ = true;

            // update
            if (clusterId >= 0)
            {
                evidence.add(ptCell, p_occupancy, numpoints);
                ptCell->addRayThrough(hit_probability);
            }
            else
            {
                ptCell->updateOccupancy(numpoints * p_occupancy, occupancy_limit);
            }
        }

        // This is copied from addDistributionToCell fro convenience; we need the pointer
        if (withinHeightLimit)
        {
            Eigen::Matrix3d cov = ndts[it]->getCov();
            int label = ndts[it]->getLabel();
            int weight = ndts[it]->getLabelWeight();

            pcl::PointXYZL pt;
            pt.x = mean[0];
            pt.y = mean[1];
            pt.z = mean[2];
            LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
            if (lz == NULL)
            {
                fprintf(stderr, "NOT LAZY GRID!!!\n");
                exit(1);
            }
            NDTCell* ptCell = NULL;
            lz->getCellAtAllocate(pt, ptCell);

            if (ptCell != NULL)
            {
                int clusterId = ptCell->getClusterId();
                // do regular update only if not part of a cluster
                bool updateOccupancy = clusterId < 0;

                double positiveOccupancy = getPositiveUpdate(ptCell);
                ptCell->updateSampleVariance(cov, mean, numpoints, updateOccupancy,
                                             positiveOccupancy, occupancy_limit, maxnumpoints);

                // FIXME: set the label
                ptCell->setLabel(label, weight);

                // debug
                ptCell->viz_updated_ = true;
                ptCell->viz_accessed_ = true;

                // update
                if (!updateOccupancy)
                {
                    evidence.add(ptCell, positiveOccupancy, numpoints);
                    ptCell->addRayHit();
                }

                if (clusterId == -1)
                {
                    initializeCluster(ptCell, true, false);
                }
            }
        }
    }

    // if(frame_ % 2 == 0)
    {
        cluster(false, true, false, 20);
    }
    frame_++;
    if (frame_ > 100)
    {
        frame_ = 0;
    }

    //  CLUSTER EVIDENCE UPDATE
    updateClusters(evidence, occupancy_limit);

    isFirstLoad_ = false;
}

int NDTMap::getNextFreeClusterId()
{
    int id = next_free_cluster_id_;
    next_free_cluster_id_++;
    return id;
}

void NDTMap::updateClusters(EvidenceContainer& allEvidence, float occupancy_limit)
{
    bool verbose = false;
    for (unsigned int i = 0; i < allEvidence.allIndices_.size(); i++)
    {
        int id = allEvidence.allIndices_[i];
        std::vector<int> indices = allEvidence.getIndices(id);

        bool found = false;
        if (id < 0)
        {
            continue;
        }

        // get total evidence
        if(verbose)
        {
            std::cout << "***************************************************************" << std::endl;
            std::cout << "update cluster: " << id << " size: " << indices.size() << std::endl;
            std::cout << std::endl;

            std::cout << "get cluster evidence **********************" << std::endl;
        }
        double totalHits;
        double totalThrough;
        double totalEvidence = clusterEvidence(allEvidence, indices, totalHits, totalThrough);

        if(verbose)
        {
            std::cout << "initial total evidence l: " << totalEvidence << " p: " << prob(totalEvidence)
                    << std::endl;
            std::cout << std::endl;

            std::cout << "update membership *************************" << std::endl;
        }
        updateMembership(allEvidence, indices, totalHits, totalThrough);
        double updatedClusterEvidence =
            clusterEvidence(allEvidence, indices, totalHits, totalThrough);
        if(verbose)
        {
            std::cout << std::endl;

            std::cout << "get cluster evidence **********************" << std::endl;

            std::cout << "updated total evidence l: " << updatedClusterEvidence
                    << " p: " << prob(updatedClusterEvidence) << std::endl;
        }

        // get cluster & update
        auto search = clusters_.find(id);
        if (search == clusters_.end())
        {
            continue;
        }
        std::vector<NDTCell*> cluster = search->second;

        if(verbose)
        {
            std::cout << std::endl;
            std::cout << "update ************************************" << std::endl;
            std::cout << "cluster size " << cluster.size() << std::endl;
        }

        for (unsigned int i = 0; i < cluster.size(); i++)
        {
            NDTCell* cell = cluster[i];

            if (cell)
            {
                double mem = cell->getClusterMembership();
                bool found = false;
                Evidence e = allEvidence.get(cell, found);
                double evidence = w_cluster_ * mem * updatedClusterEvidence;
                if (found)
                {
                    evidence = w_own_ * e.totalLikelihood_ + evidence;
                }

                float origOcc = cell->getOccupancy();

                cell->updateOccupancy(evidence, occupancy_limit);
                if (cell->getOccupancy() < 0)
                {
                    cell->hasGaussian_ = false;
                }

                if (cell->getClusterMembership() < 0.25)
                {
                    removeFromCluster(cell);
                    initializeCluster(cell);
                }

                if(verbose)
                {
                    std::cout << "mem: " << std::fixed << std::setprecision(4) << std::setw(8) << mem;
                    std::cout << " evidence l: " << std::fixed << std::setprecision(4) << std::setw(8)
                            << evidence;
                    std::cout << " p: " << std::fixed << std::setprecision(4) << std::setw(8)
                            << prob(evidence);
                    std::cout << " occ from " << std::fixed << std::setprecision(4) << std::setw(8)
                            << origOcc;
                    std::cout << " to " << std::fixed << std::setprecision(4) << std::setw(8)
                            << cell->getOccupancy();
                    std::cout << std::endl;
                }
            }
        }
        if(verbose)
        {
            std::cout << "cluster update done!" << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
        }
    }
}

double NDTMap::clusterEvidence(const EvidenceContainer& evidence,
                               const std::vector<int>& indices,
                               double& totalHits,
                               double& totalThrough)
{
    bool verbose = false;

    double totalEvidence = 0;
    totalHits = 0;
    totalThrough = 0;
    double cnt = (double)indices.size();
    for (unsigned int i = 0; i < indices.size(); i++)
    {
        int id = indices[i];
        Evidence e = evidence[id];
        if (e.cell_)
        {
            double is = totalEvidence;
            double mem = e.cell_->getClusterMembership();
            double increment = e.totalLikelihood_ * mem;
            if (std::isnan(increment))
            {
                increment = 0;
            }
            if (std::isinf(increment))
            {
                increment = logOdds(0.99);
            }
            totalEvidence += increment;
            totalHits += e.cell_->getRayHits();
            totalThrough += e.cell_->getRayThroughs();

            if(verbose)
            {
                std::cout << std::setw(4) << std::setfill(' ') << i;
                std::cout << ": init " << std::fixed << std::setprecision(4) << std::setw(8) << is;
                std::cout << " l " << std::fixed << std::setprecision(4) << std::setw(8) << increment;
                std::cout << " mem " << std::fixed << std::setprecision(4) << std::setw(8) << mem;
                std::cout << " totalEvidence " << std::fixed << std::setprecision(4) << std::setw(8)
                        << totalEvidence;
                std::cout << std::endl;
            }
        }
    }

    return totalEvidence;
}

void NDTMap::updateMembership(const EvidenceContainer& evidence,
                              const std::vector<int>& indices,
                              double totalHits,
                              double totalThrough)
{
    for (unsigned int i = 0; i < indices.size(); i++)
    {
        int id = indices[i];
        Evidence e = evidence[id];
        if (e.cell_)
        {
            double total = totalHits + totalThrough;

            // no evidence
            if (total == 0)
            {
                continue;
            }
            double p_hit = totalHits / total;
            double p_thru = totalThrough / total;

            if (p_hit == 0)
            {
                p_hit = 1.0 / 1000;
            }
            if (p_thru == 0)
            {
                p_thru = 1.0 / 1000;
            }

            double hits = e.cell_->getRayHits();
            double thrus = e.cell_->getRayThroughs();
            double N = hits + thrus;

            // no evidence
            if (N == 0)
            {
                continue;
            }

            double stat = N * (((hits / N - p_hit) * (hits / N - p_hit) / p_hit) +
                               ((thrus / N - p_thru) * (thrus / N - p_thru) / p_thru));

            double mem;
            if (stat == 0)
            {
                mem = 1;
            }
            else
            {
                boost::math::chi_squared distribution(1.0);
                mem = boost::math::pdf(distribution, stat);
                mem = clamp(mem, 0, 1);
            }

            e.cell_->setClusterMembership(mem);
        }
    }
}

//  █████╗ ██████╗ ██████╗     ███╗   ███╗███████╗ █████╗ ███████╗
// ██╔══██╗██╔══██╗██╔══██╗    ████╗ ████║██╔════╝██╔══██╗██╔════╝
// ███████║██║  ██║██║  ██║    ██╔████╔██║█████╗  ███████║███████╗
// ██╔══██║██║  ██║██║  ██║    ██║╚██╔╝██║██╔══╝  ██╔══██║╚════██║
// ██║  ██║██████╔╝██████╔╝    ██║ ╚═╝ ██║███████╗██║  ██║███████║
// ╚═╝  ╚═╝╚═════╝ ╚═════╝     ╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚══════╝

/**
 * Adds one measurement to the map using NDT-OM update step
 * @return true if an inconsistency was detected
 */
bool NDTMap::addMeasurement(const Eigen::Vector3d& origin,
                            pcl::PointXYZL endpoint,
                            double classifierTh,
                            double maxz,
                            double sensor_noise)
{

    if (index_ == NULL)
    {
        return false;
    }

    bool retval = false;
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    double centerX, centerY, centerZ;
    lz->getCenter(centerX, centerY, centerZ);
    double cellSizeX, cellSizeY, cellSizeZ;
    lz->getCellSize(cellSizeX, cellSizeY, cellSizeZ);
    int sizeX, sizeY, sizeZ;
    lz->getGridSize(sizeX, sizeY, sizeZ);

    double min1 = std::min(cellSizeX, cellSizeY);
    double min2 = std::min(cellSizeZ, cellSizeY);

    // Select the smallest resolution
    double resolution = std::min(min1, min2);

    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    NDTCell* ptCell = NULL;

    pcl::PointXYZL pt;
    pcl::PointXYZL po;
    po.x = origin(0), origin(1), origin(2);

    Eigen::Vector3d diff;
    diff << endpoint.x - origin(0), endpoint.y - origin(1), endpoint.z - origin(2);

    double l = diff.norm();
    if (l > 200)
    {
        fprintf(stderr, "addMeasurement::Very long distance (%lf) :( \n", l);
        return false;
    }
    if (resolution < 0.01)
    {
        fprintf(stderr, "Resolution very very small (%lf) :( \n", resolution);
        return false;
    }

    int NN = l / (resolution);
    if (NN <= 0)
        return false;
    diff = diff / (float)NN;
    bool updatePositive = true;
    if (endpoint.z > maxz)
    {
        return false;
    }

    int idxo = 0, idyo = 0, idzo = 0;
    for (int i = 0; i < NN - 2; i++)
    {
        pt.x = origin(0) + ((float)(i + 1)) * diff(0);
        pt.y = origin(1) + ((float)(i + 1)) * diff(1);
        pt.z = origin(2) + ((float)(i + 1)) * diff(2);
        int idx, idy, idz;

        idx = (int)(((pt.x - centerX) / cellSizeX + 0.5) + sizeX / 2);
        idy = (int)(((pt.y - centerY) / cellSizeY + 0.5) + sizeY / 2);
        idz = (int)(((pt.z - centerZ) / cellSizeZ + 0.5) + sizeZ / 2);

        // We only want to check every cell once, so
        // increase the index if we are still in the same cell
        if (idx == idxo && idy == idyo && idz == idzo)
        {
            continue;
        }
        else
        {
            idxo = idx;
            idyo = idy, idzo = idz;
        }
        // Check the validity of the index
        // if(idx < sizeX && idy < sizeY && idz < sizeZ && idx >=0 && idy >=0 && idz >=0)
        //{
        //     ptCell = dynamic_cast<NDTCell<PointT> *>  (lz->getCellAt(idx,idy,idz));
        //     //dataArray[idx][idy][idz]);
        lz->getNDTCellAt(idx, idy, idz, ptCell);
        //}
        // else
        //{
        //    continue;
        //}

        if (ptCell != NULL)
        {
            double l2target = 0;

            if (ptCell->hasGaussian_)
            {
                Eigen::Vector3d out, pend, vpt;

                pend << endpoint.x, endpoint.y, endpoint.z; ///< endpoint

                double lik = ptCell->computeMaximumLikelihoodAlongLine(po, pt, out);
                double dist = (origin - out).norm();
                if (dist > l)
                    continue; ///< don't accept points further than the measurement

                l2target = (out - pend).norm(); ///< distance to endpoint
                // double thr =exp(-0.5*(l2target*l2target)/(sensor_noise*sensor_noise)); ///This is
                // the probability of max lik point being endpoint
                // ptCell->updateEmpty(lik*(1-thr),l2target);

                double sigma_dist = 0.5 * (dist / 30.0); // test for distance based sensor noise
                double snoise = sigma_dist + sensor_noise;
                double thr = exp(
                    -0.5 * (l2target * l2target) /
                    (snoise * snoise)); // This is the probability of max lik point being endpoint
                lik *= (1.0 - thr);
                lik = -0.2 * lik + 0.5; // Evidence value for empty - alpha * p(x);
                double logoddlik = logOdds(lik);
                ptCell->updateEmpty(logoddlik, l2target);

                /*
                if(lik>thr){
                    retval = true;
                    ptCell->updateEmpty(lik,l2target);
                }*/
            }
            else
            {
                ptCell->updateEmpty(-0.1, l2target); // The cell does not have gaussian, so we mark
                                                     // that we saw it empty...
            }
        }
        else
        {
            index_->addPoint(pt); // Add fake point to initialize!
        }
    }

    if (updatePositive)
        index_->addPoint(endpoint);

    isFirstLoad_ = false;

    return retval;
}

// ██╗      ██████╗  █████╗ ██████╗     ██████╗  ██████╗
// ██║     ██╔═══██╗██╔══██╗██╔══██╗    ██╔══██╗██╔════╝
// ██║     ██║   ██║███████║██║  ██║    ██████╔╝██║
// ██║     ██║   ██║██╔══██║██║  ██║    ██╔═══╝ ██║
// ███████╗╚██████╔╝██║  ██║██████╔╝    ██║     ╚██████╗
// ╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚═════╝     ╚═╝      ╚═════╝

void NDTMap::loadPointCloud(const pcl::PointCloud<pcl::PointXYZL>& pc,
                            const std::vector<std::vector<size_t>>& indices)
{

    //    loadPointCloud(pc);
    // Specific function related to CellVector
    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        for (size_t i = 0; i < indices.size(); i++)
        {
            cl->addCellPoints(pc, indices[i]);
        }
    }
    else
    {
        // ERR("loading point clouds using indices are currently supported in CellVector index_.");
    }
}

// ██████╗
// ╚════██╗
//  █████╔╝
// ██╔═══╝
// ███████╗
// ╚══════╝

/**
 * loadPointCloud - You can call this if you are only interested in dealing with one scan
 * without need for fusing several ones or representing empty space and occupancy
 *
 * Otherwise you should always call addPointCloud (or if you don't want occupancy then
 * addPointCloudSimple)
 *
 * \param pc the PointCloud that is to be loaded
 * \note every subsequent call will destroy the previous map!
 */
void NDTMap::loadPointCloud(const pcl::PointCloud<pcl::PointXYZL>& pc, double range_limit)
{
    if (index_ != NULL)
    {
        // std::cout<<"CLONE INDEX\n";
        SpatialIndex* si = index_->clone();
        // cout<<"allocating index\n";
        if (!isFirstLoad_)
        {
            // std::cout<<"deleting old index\n";
            delete index_;
        }

        isFirstLoad_ = false;
        index_ = si;
    }
    else
    {
        // NULL index in constructor, abort!
        // ERR("constructor must specify a non-NULL spatial index\n");
        return;
    }

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "Unfortunately This works only with Lazygrid!\n");
        exit(1);
    }

    if (index_ == NULL)
    {
        // ERR("Problem creating index, unimplemented method\n");
        return;
    }

    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = pc.points.begin();
    if (guess_size_)
    {
        double maxDist = 0; //, distCeil = 200;

        Eigen::Vector3d centroid(0, 0, 0);
        int npts = 0;
        while (it != pc.points.end())
        {
            Eigen::Vector3d d;
            if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
            {
                it++;
                continue;
            }
            d << it->x, it->y, it->z;
            if (range_limit > 0)
            {
                if (d.norm() > range_limit)
                {
                    it++;
                    continue;
                }
            }
            centroid += d;
            it++;
            npts++;
        }

        centroid /= (double)npts;
        double maxz = -1000, minz = 10000;
        // Eigen::Vector4f centroid(0,0,0,0);
        // pcl::compute3DCentroid(pc,centroid);

        // compute distance to furthest point
        it = pc.points.begin();
        while (it != pc.points.end())
        {
            Eigen::Vector3d d;
            if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
            {
                it++;
                continue;
            }
            if (range_limit > 0)
            {
                d << it->x, it->y, it->z;
                if (d.norm() > range_limit)
                {
                    it++;
                    continue;
                }
            }
            d << centroid(0) - it->x, centroid(1) - it->y, centroid(2) - it->z;
            double dist = d.norm();
            maxDist = (dist > maxDist) ? dist : maxDist;
            maxz = ((centroid(2) - it->z) > maxz) ? (centroid(2) - it->z) : maxz;
            minz = ((centroid(2) - it->z) < minz) ? (centroid(2) - it->z) : minz;
            it++;
        }
        // cout<<"Points = " <<pc.points.size()<<" maxDist = "<<maxDist<<endl;
        NDTCell* ptCell = new NDTCell();
        index_->setCellType(ptCell);
        delete ptCell;
        index_->setCenter(centroid(0), centroid(1), centroid(2));

        if (map_sizex > 0 && map_sizey > 0 && map_sizez > 0)
        {
            index_->setSize(map_sizex, map_sizey, map_sizez);
        }
        else
        {
            index_->setSize(4 * maxDist, 4 * maxDist, 3 * (maxz - minz));
        }
    }
    else
    {
        // set sizes
        NDTCell* ptCell = new NDTCell();
        index_->setCellType(ptCell);
        delete ptCell;
        index_->setCenter(centerx, centery, centerz);
        if (map_sizex > 0 && map_sizey > 0 && map_sizez > 0)
        {
            index_->setSize(map_sizex, map_sizey, map_sizez);
        }
    }

    //    ROS_INFO("centroid is %f,%f,%f", centroid(0),centroid(1),centroid(2));
    //    ROS_INFO("maxDist is %lf", maxDist);

    it = pc.points.begin();
    while (it != pc.points.end())
    {
        Eigen::Vector3d d;
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }
        if (range_limit > 0)
        {
            d << it->x, it->y, it->z;
            if (d.norm() > range_limit)
            {
                it++;
                continue;
            }
        }
        index_->addPoint(*it);
        NDTCell* ptCell;
        lz->getNDTCellAt(*it, ptCell);

        if (ptCell != NULL)
        {
            //    std::cerr<<"adding to update set\n";
            update_set.insert(ptCell);
        }

        it++;
    }

    isFirstLoad_ = false;
}

// ██████╗ ███████╗██████╗ ████████╗██╗  ██╗
// ██╔══██╗██╔════╝██╔══██╗╚══██╔══╝██║  ██║
// ██║  ██║█████╗  ██████╔╝   ██║   ███████║
// ██║  ██║██╔══╝  ██╔═══╝    ██║   ██╔══██║
// ██████╔╝███████╗██║        ██║   ██║  ██║
// ╚═════╝ ╚══════╝╚═╝        ╚═╝   ╚═╝  ╚═╝

void NDTMap::loadDepthImage(const cv::Mat& depthImage, DepthCamera<pcl::PointXYZL>& cameraParams)
{
    pcl::PointCloud<pcl::PointXYZL> pc;
    cameraParams.convertDepthImageToPointCloud(depthImage, pc);
    this->loadPointCloud(pc);
}

pcl::PointCloud<pcl::PointXYZL> NDTMap::loadDepthImageFeatures(
    const cv::Mat& depthImage,
    std::vector<cv::KeyPoint>& keypoints,
    size_t& supportSize,
    double maxVar,
    DepthCamera<pcl::PointXYZL>& cameraParams,
    bool estimateParamsDI,
    bool nonMean)
{
    std::vector<cv::KeyPoint> good_keypoints;
    Eigen::Vector3d mean;
    pcl::PointXYZL mn;
    pcl::PointCloud<pcl::PointXYZL> cloudOut;
    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl == NULL)
    {
        std::cerr << "wrong index type!\n";
        return cloudOut;
    }
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        if (!estimateParamsDI)
        {
            pcl::PointCloud<pcl::PointXYZL> points;
            pcl::PointXYZL center;
            cameraParams.computePointsAtIndex(depthImage, keypoints[i], supportSize, points,
                                              center);
            NDTCell* ndcell = new NDTCell();
            typename pcl::PointCloud<pcl::PointXYZL>::iterator it = points.points.begin();
            while (it != points.points.end())
            {
                if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
                {
                    it++;
                    continue;
                }
                ndcell->addPoint(*it);
                it++;
            }
            ndcell->computeGaussian();
            if (ndcell->hasGaussian_)
            {
                Eigen::Vector3d evals = ndcell->getEvals();
                if (sqrt(evals(2)) < maxVar)
                {
                    if (nonMean)
                    {
                        if (std::isnan(center.x) || std::isnan(center.y) || std::isnan(center.z))
                        {
                            continue;
                        }
                        mn = center;
                    }
                    else
                    {
                        mean = ndcell->getMean();
                        mn.x = mean(0);
                        mn.y = mean(1);
                        mn.z = mean(2);
                    }
                    cloudOut.points.push_back(mn);
                    ndcell->setCenter(mn);
                    cl->addCell(ndcell);
                    good_keypoints.push_back(keypoints[i]);
                }
            }
        }
        else
        {
            assert(nonMean = false); // Not implemented / used.
            Eigen::Vector3d mean;
            Eigen::Matrix3d cov;
            cameraParams.computeParamsAtIndex(depthImage, keypoints[i], supportSize, mean, cov);
            NDTCell* ndcell = new NDTCell();
            ndcell->setMean(mean);
            ndcell->setCov(cov);

            if (ndcell->hasGaussian_)
            {
                Eigen::Vector3d evals = ndcell->getEvals();
                // std::cout<<evals.transpose()<<std::endl;
                if (sqrt(evals(2)) < maxVar)
                {
                    mean = ndcell->getMean();
                    mn.x = mean(0);
                    mn.y = mean(1);
                    mn.z = mean(2);
                    cloudOut.points.push_back(mn);
                    ndcell->setCenter(mn);
                    cl->addCell(ndcell);
                    good_keypoints.push_back(keypoints[i]);
                }
            }
        }
    }

    // ORU_TODO keypoints??
    keypoints = good_keypoints;
    return cloudOut;
}

//  ██████╗███████╗███╗   ██╗████████╗██████╗  ██████╗ ██╗██████╗
// ██╔════╝██╔════╝████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║██╔══██╗
// ██║     █████╗  ██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║██║  ██║
// ██║     ██╔══╝  ██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║██║  ██║
// ╚██████╗███████╗██║ ╚████║   ██║   ██║  ██║╚██████╔╝██║██████╔╝
//  ╚═════╝╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═╝╚═════╝

/**
 * loadPointCloudCentroid - A special load function to enable the matching of centroids (create
 * alligned maps) \param pc the PointCloud that is to be loaded \note every subsequent call will
 * destroy the previous map!
 */
void NDTMap::loadPointCloudCentroid(const pcl::PointCloud<pcl::PointXYZL>& pc,
                                    const Eigen::Vector3d& origin,
                                    const Eigen::Vector3d& old_centroid,
                                    const Eigen::Vector3d& map_size,
                                    double range_limit)
{

    if (index_ != NULL)
    {
        SpatialIndex* si = index_->clone();
        if (!isFirstLoad_)
            delete index_;

        isFirstLoad_ = false;
        index_ = si;
    }
    else
    {
        return;
    }

    if (index_ == NULL)
        return; // Should never happen!

    NDTCell* ptCell = new NDTCell();
    index_->setCellType(ptCell);
    delete ptCell;

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "Unfortunately This works only with Lazygrid!\n");
        exit(1);
    }

    Eigen::Vector3d diff = origin - old_centroid; //-origin;
    double cx = 0, cy = 0, cz = 0;
    lz->getCellSize(cx, cy, cz);

    // How many cell to each direction is the new origin from old one
    Eigen::Vector3d centroid;
    centroid(0) = old_centroid(0) + floor(diff(0) / cx) * cx;
    centroid(1) = old_centroid(1) + floor(diff(1) / cy) * cy;
    centroid(2) = old_centroid(2) + floor(diff(2) / cz) * cz;

    index_->setCenter(centroid(0), centroid(1), centroid(2));
    // index_->setCenter(origin(0),origin(1),origin(2));
    index_->setSize(map_size(0), map_size(1), map_size(2));
    // lz->initializeAll();

    // temporarily commented fprintf(stderr,"centroid is %lf,%lf,%lf (origin: %lf %lf %lf) (map_size
    // %lf %lf %lf) N=%d", centroid(0),centroid(1),centroid(2), origin(0),origin(1),origin(2),
    // map_size(0), map_size(1), map_size(2),(int)pc.size());
    //  ROS_INFO("centroid is %f,%f,%f", centroid(0),centroid(1),centroid(2));
    //  ROS_INFO("maxDist is %lf", maxDist);

    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = pc.points.begin();

    while (it != pc.points.end())
    {
        Eigen::Vector3d d;
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }

        //        std::cout << "centoroid add point [" << it->x << "," << it->y << "," <<it->z
        //        <<std::endl;

        if (range_limit > 0)
        {
            d << it->x, it->y, it->z;
            d = d - origin;
            if (d.norm() > range_limit)
            {
                it++;
                continue;
            }
        }

        // fprintf(stderr,"HEP!");
        index_->addPoint(*it);
        NDTCell* ptCell = NULL;
        lz->getNDTCellAt(*it, ptCell);

        if (ptCell != NULL)
        {
            update_set.insert(ptCell);
        }

        it++;
    }

    isFirstLoad_ = false;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

// ███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗
// ██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔══██╗██║
// █████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████║██║
// ██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║██╔══██║██║
// ██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║██║  ██║███████╗
// ╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝

double NDTMap::logOdds(const double p) const
{
    return log((p) / (1.0 - p));
}

double NDTMap::prob(const double l) const
{
    if (l > 200)
    {
        return 1;
    }
    else
    {
        return exp(l) / (1.0 + exp(l));
    }
}

double NDTMap::saturate(const double x) const
{
    double L = 5.0;
    double k = 0.5;
    double x0 = 0.0;
    return (2 * L) / (1 + exp(-k * (x - x0))) - L;
}

int NDTMap::getMaxClusterId() const
{
    if (clusters_.size() > 0)
    {
        return clusters_.rbegin()->first;
    }
    else
    {
        return -1;
    }
}

void NDTMap::computeMaximumLikelihoodPointCloudWithRangePairs(
    const Eigen::Vector3d& origin,
    const pcl::PointCloud<pcl::PointXYZL>& pc,
    const Eigen::Vector3d& virtualOrigin,
    pcl::PointCloud<pcl::PointXYZL>& pc_out,
    std::vector<std::pair<double, double>>& ranges,
    double max_range = 130.) const
{
    pc_out.clear();
    if (isFirstLoad_ || index_ == NULL)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = pc.points.begin();

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);
    NDTCell* ptCell = NULL;

    // Step through the points, compute the directions and for now do the full line tracing...
    std::vector<NDTCell*> cells;
    bool updatePositive = true;

    while (it != pc.points.end())
    {
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }

        Eigen::Vector3d diff;
        diff << it->x - origin(0), it->y - origin(1), it->z - origin(2);
        double l = diff.norm();

        cells.clear();

        // Trace line uses the diff to check that we never go beyond the diff value...
        // Hacky for now but force the diff to be a bit larger -> 3 meters...
        diff *= ((l + 3.) / l);
        if (!lz->traceLine(virtualOrigin, *it, diff, 1000.0, cells))
        {
            it++;
            continue;
        }

        for (unsigned int i = 0; i < cells.size(); i++)
        {
            ptCell = cells[i];
            if (ptCell != NULL)
            {
                double l2target = 0;
                if (ptCell->hasGaussian_)
                {
                    Eigen::Vector3d vout;
                    pcl::PointXYZL out;
                    vout << out.x, out.y, out.z;
                    double lik = ptCell->computeMaximumLikelihoodAlongLine(po, *it, out);
                    // std::cerr << "lik : " << lik << std::endl;
                    if (lik > 0.)
                    {
                        ranges.push_back(std::pair<double, double>(l, (vout - origin).norm()));
                        pc_out.push_back(out);
                    }
                }
            }
        }
        it++;
    }
}

void NDTMap::computeConflictingPoints(const Eigen::Vector3d& origin,
                                      const pcl::PointCloud<pcl::PointXYZL>& pc,
                                      pcl::PointCloud<pcl::PointXYZL>& pc_out,
                                      pcl::PointCloud<pcl::PointXYZL>& pc2_out,
                                      double likelihoodFactor) const
{
    pc_out.clear();
    pc2_out.clear();
    if (isFirstLoad_ || index_ == NULL)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZL>::const_iterator it = pc.points.begin();

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);
    NDTCell* ptCell = NULL;

    while (it != pc.points.end())
    {
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        {
            it++;
            continue;
        }

        Eigen::Vector3d diff;
        diff << it->x - origin(0), it->y - origin(1), it->z - origin(2);
        double l = diff.norm();

        // Get the cell
        ptCell = lz->getClosestNDTCell(*it);
        if (ptCell == NULL)
        {
            pc_out.push_back(*it);
            it++;
            continue;
        }
        // if (ptCell->getOccupancyRescaled() < 0.8) {
        //     pc_out.push_back(*it);
        //     it++;
        //     continue;
        // }

        if (!ptCell->hasGaussian_)
        {
            pc_out.push_back(*it);
            it++;
            continue;
        }

        double lik = ptCell->getLikelihood(*it);
        //        std::cerr << "lik : " << lik << " " << std::flush;
        lik = (lik < 0) ? 0 : lik;
        Eigen::Vector3d vout, pt;
        pcl::PointXYZL out;
        vout << out.x, out.y, out.z;
        pt << it->x, it->y, it->z;
        double lik_trace = ptCell->computeMaximumLikelihoodAlongLine(po, *it, out);
        //        std::cerr << " lik : " << lik << " lik_trace : " << lik_trace << " lik/lik_trace :
        //        " << lik/lik_trace << std::endl;

        // if (lik / lik_trace < likelihoodFactor) {
        //     pc_out.push_back(*it);
        // }
        //        if (lik < likelihoodFactor) {
        //        std::cerr << " " << norm << std::flush;
        double norm = (pt - vout).norm();
        //        if (norm > 0.8)
        // if (fabs(pt(2) - out(2)) > 0.2)
        // {
        //     pc_out.push_back(*it);
        //     pc2_out.push_back(pcl::PointXYZL(out(0),out(1),out(2)));
        // }
        pc_out.push_back(out);
        it++;
    }
}

void NDTMap::computeMaximumLikelihoodRanges(const Eigen::Vector3d& origin,
                                            const Eigen::VectorXd& rawRanges,
                                            const std::vector<Eigen::Vector3d>& dirs,
                                            Eigen::VectorXd& ranges) const
{

    if (isFirstLoad_ || index_ == NULL)
    {
        return;
    }

    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    if (lz == NULL)
    {
        fprintf(stderr, "NOT LAZY GRID!!!\n");
        exit(1);
    }
    pcl::PointXYZL po;
    po.x = origin(0);
    po.y = origin(1);
    po.z = origin(2);
    NDTCell* ptCell = NULL;

    // Step through the points, compute the directions and for now do the full line tracing...
    std::vector<NDTCell*> cells;
    bool updatePositive = true;

    double resolution = this->getSmallestCellSizeInMeters();
    ranges.resize(dirs.size());
    ranges.setZero();

    for (unsigned int i = 0; i < dirs.size(); i++)
    {
        cells.clear();

        Eigen::Vector3d p1 = origin;
        Eigen::Vector3d p2 = origin + dirs[i] * 100.;
        // Eigen::Vector3d p2 = origin+(rawRanges[i]+resolution)*dirs[i];
        if (!lz->traceLine(p1, p2, dirs[i] * 100., 1000.0, cells))
        {
            continue;
        }

        double max_lik = 0.;
        double range = 0.;
        for (unsigned int j = 0; j < cells.size(); j++)
        {
            ptCell = cells[j];
            if (ptCell != NULL)
            {
                if (ptCell->hasGaussian_)
                {
                    Eigen::Vector3d out;
                    double lik = ptCell->computeMaximumLikelihoodAlongLine(p1, p2, out);

                    if (lik > max_lik)
                    {
                        max_lik = lik;
                        range = (out - origin).norm();
                        ranges[i] = range;
                    }
                }
            }
        }

        //        std::cout << "ranges[" << i << "] : " << ranges[i] << "   -   " << rawRanges[i] <<
        //        std::endl;
    }
}

/** Helper function, computes the  NDTCells
 */
void NDTMap::computeNDTCells(int cellupdatemode,
                             unsigned int maxnumpoints,
                             float occupancy_limit,
                             Eigen::Vector3d origin,
                             double sensor_noise)
{
    CellVector* cv = dynamic_cast<CellVector*>(index_);
    conflictPoints.clear();
    typename std::set<NDTCell*>::iterator it = update_set.begin();

    // FIXME: debug statistics
    int nonzero = 0;
    int put = 0;

    // std::cout << "update set size " << update_set.size() << std::endl;

    while (it != update_set.end())
    {
        NDTCell* cell = *it;
        if (cell != NULL)
        {
            // calculate label
            if (cell->points_.size() > 0)
            {
                put++;
                cell->calculateLabel();
                int label = cell->getLabel();
                nonzero = label != 0 ? nonzero + 1 : nonzero;
            }

            // compute gaussian
            //	        std::cout << "Compute gaussians " << std::endl;
            double positiveOccupancy = getPositiveUpdate(cell);
            cell->computeGaussian(cellupdatemode, maxnumpoints, positiveOccupancy, occupancy_limit,
                                  origin, sensor_noise);

            // Process the conflict points
            if (cell->points_.size() > 0)
            {
                for (unsigned int i = 0; i < cell->points_.size(); i++)
                    conflictPoints.push_back(cell->points_[i]);
                cell->points_.clear();
            }

            // FIXME: why the cell vector is needed?
            //  Set the mean to the cell's centre.
            if (cv != NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                pcl::PointXYZL pt;
                pt.x = mean[0];
                pt.y = mean[1];
                pt.z = mean[2];

                cell->setCenter(pt);
            }
        }
        else
        {
        }
        it++;
    }
    update_set.clear();

    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        cl->initKDTree();
    }
}

/**
 * Computes the normaldistribution parameters and leaves the points a
 */
void NDTMap::computeNDTCellsSimple(bool keepPoints)
{
    CellVector* cv = dynamic_cast<CellVector*>(index_);

    conflictPoints.clear();

    typename SpatialIndex::CellVectorItr it = index_->begin();

    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell != NULL)
        {
            cell->computeGaussianSimple();

            if (cv != NULL)
            {
                // Set the mean to the cell's centre.
                Eigen::Vector3d mean = cell->getMean();
                pcl::PointXYZL pt;
                pt.x = mean[0];
                pt.y = mean[1];
                pt.z = mean[2];

                cell->setCenter(pt);
            }
            if (!keepPoints)
            {
                cell->points_.clear();
            }
        }
        else
        {
        }
        it++;
    }

    CellVector* cl = dynamic_cast<CellVector*>(index_);
    if (cl != NULL)
    {
        cl->initKDTree();
    }
}

NDTMap* NDTMap::pseudoTransformNDTMap(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> T)
{
    NDTMap* map = new NDTMap(new CellVector(), true);
    CellVector* idx = dynamic_cast<CellVector*>(map->getMyIndex());
    typename SpatialIndex::CellVectorItr it = index_->begin();

    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell->hasGaussian_)
        {

            Eigen::Vector3d mean = cell->getMean();
            Eigen::Matrix3d cov = cell->getCov();
            mean = T * mean;
            // NOTE: The rotation of the covariance fixed by Jari 6.11.2012
            cov = T.rotation() * cov * T.rotation().transpose();
            NDTCell* nd = (NDTCell*)cell->clone();
            nd->setMean(mean);
            nd->setCov(cov);
            idx->addNDTCell(nd);
        }
        it++;
    }
    return map;
}

std::vector<NDTCell*> NDTMap::pseudoTransformNDT(
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> T) const
{

    std::vector<NDTCell*> ret;
    typename SpatialIndex::CellVectorConstItr it = index_->begin();
    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell != NULL)
        {
            if (cell->hasGaussian_)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = T * mean;
                // NOTE: The rotation of the covariance fixed by Jari 6.11.2012
                cov = T.rotation() * cov * T.rotation().transpose();
                NDTCell* nd = (NDTCell*)cell->clone();
                nd->setMean(mean);
                nd->setCov(cov);
                ret.push_back(nd);
            }
        }
        else
        {
            // ERR("problem casting cell to NDT!\n");
        }
        it++;
    }
    return ret;
}

int NDTMap::numberOfActiveCells()
{
    int ret = 0;
    if (index_ == NULL)
        return ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        if ((*it)->hasGaussian_)
        {
            ret++;
        }
        it++;
    }
    return ret;
}

int NDTMap::numberOfActiveCells() const
{
    int ret = 0;
    if (index_ == NULL)
        return ret;
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        if ((*it)->hasGaussian_)
        {
            ret++;
        }
        it++;
    }
    return ret;
}

bool NDTMap::insertCell(NDTCell cell)
{
    LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
    gr->insertCell(*cell.copy());
}
std::string NDTMap::ToString()
{
    std::stringstream ss;
    if (index_ != NULL)
        ss << "NDTMap: index= \n" << index_->ToString() << std::endl;
    return ss.str();
}
double L2_Score(NDTMap* target, std::vector<NDTCell*> source)
{

    double score = 1;
    if (source.size() == 0)
    {
        fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
        return 0;
    }

    for (int n = 0; n < source.size(); n++)
    {
        Eigen::Vector3d m = source[n]->getMean();

        NDTCell* cell;
        pcl::PointXYZL p;
        p.x = m[0];
        p.y = m[1];
        p.z = m[2];

        if (target->getCellAtPoint(p, cell))
        {
            if (cell == NULL)
                continue;
            if (cell->hasGaussian_)
            {
                Eigen::Matrix3d covCombined = cell->getCov() + source[n]->getCov();
                Eigen::Matrix3d icov;
                bool exists;
                double det = 0;
                covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                if (!exists)
                    continue;
                double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
                if (l * 0 != 0)
                    continue;
                score += 0.1 + 0.9 * exp(-0.05 * l / 2.0);
            }
        }
    }
    return score;
}
double L2_score_corrected(NDTMap* target, std::vector<NDTCell*> source)
{

    double score = 0;
    if (source.size() == 0)
    {
        fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
        return 0;
    }
    double sum_area = 0;

    for (int n = 0; n < source.size(); n++)
    {
        Eigen::Vector3d m = source[n]->getMean();

        NDTCell* cell;
        pcl::PointXYZL p;
        p.x = m[0];
        p.y = m[1];
        p.z = m[2];

        if (target->getCellAtPoint(p, cell))
        {
            if (cell == NULL)
                continue;
            if (cell->hasGaussian_)
            {
                Eigen::Matrix3d covCombined = cell->getCov() + source[n]->getCov();
                Eigen::Matrix3d icov;
                bool exists;
                double det = 0;
                covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                if (!exists)
                    continue;
                double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
                if (l * 0 != 0)
                    continue;
                score += exp(-0.02 * l / 2.0); ///(cell->getCov().determinant() +
                                               // source[n]->getCov().determinant()); very noisy
            }
        }
    }

    return score;
}
double Bhattacharyya(NDTMap* target, std::vector<NDTCell*> source)
{
    double score = 0;
    if (source.size() == 0)
    {
        fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
        return 0;
    }

    for (int n = 0; n < source.size(); n++)
    {
        Eigen::Vector3d m = source[n]->getMean();

        NDTCell* cell;
        pcl::PointXYZL p;
        p.x = m[0];
        p.y = m[1];
        p.z = m[2];

        if (target->getCellAtPoint(p, cell))
        {
            if (cell == NULL)
                continue;
            if (cell->hasGaussian_)
            {
                Eigen::Matrix3d covCombined = cell->getCov() + source[n]->getCov();
                Eigen::Matrix3d icov;
                bool exists;
                double det = 0;
                covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                if (!exists)
                    continue;
                double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
                if (l * 0 != 0)
                    continue;

                double d_sigma1 = cell->getCov().determinant();
                double d_sigma2 = source[n]->getCov().determinant();
                double bd = 1 / 8.0 * l + 0.5 * log(det / sqrt(d_sigma1 + d_sigma2));
                score += exp(-0.05 * (bd) / 2);
                // score +=  exp(-0.05*l/2.0);
            }
        }
    }

    return score;
}

std::pair<int, int> NDTMap::removeVoxelsWithListedLabels(const std::vector<int>& labels)
{
    std::vector<NDTCell*> cells = getAllCells();

    int all = cells.size();
    int changed = 0;

    for (unsigned int i = 0; i < cells.size(); i++)
    {
        NDTCell* cell = cells[i];
        int label = cell->getLabel();
        if (std::find(labels.begin(), labels.end(), label) != labels.end())
        {
            cell->InitializeVariables();
            changed++;
        }
    }
    return std::pair<int, int>(changed, all);
}

int NDTMap::countListedLabels(const std::vector<int>& labels) const
{
    std::vector<NDTCell*> cells = getAllCells();
    int count = 0;

    for (unsigned int i = 0; i < cells.size(); i++)
    {
        NDTCell* cell = cells[i];
        int label = cell->getLabel();
        if (std::find(labels.begin(), labels.end(), label) != labels.end())
        {
            count++;
        }
    }

    return count;
}

/**
 * \param guess_size try to guess the size based on point cloud. Otherwise use pre-set map size
 */
void NDTMap::guessSize(float cenx, float ceny, float cenz, float sizex, float sizey, float sizez)
{
    guess_size_ = true;
    centerx = cenx;
    centery = ceny;
    centerz = cenz;
    map_sizex = sizex;
    map_sizey = sizey;
    map_sizez = sizez;
}

//  ██████╗ ██████╗ ███╗   ███╗██████╗  █████╗ ██████╗ ██╗███████╗ ██████╗ ███╗   ██╗
// ██╔════╝██╔═══██╗████╗ ████║██╔══██╗██╔══██╗██╔══██╗██║██╔════╝██╔═══██╗████╗  ██║
// ██║     ██║   ██║██╔████╔██║██████╔╝███████║██████╔╝██║███████╗██║   ██║██╔██╗ ██║
// ██║     ██║   ██║██║╚██╔╝██║██╔═══╝ ██╔══██║██╔══██╗██║╚════██║██║   ██║██║╚██╗██║
// ╚██████╗╚██████╔╝██║ ╚═╝ ██║██║     ██║  ██║██║  ██║██║███████║╚██████╔╝██║ ╚████║
//  ╚═════╝ ╚═════╝ ╚═╝     ╚═╝╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝╚══════╝ ╚═════╝ ╚═╝  ╚═══╝

ComparisonResult NDTMap::compare(const NDTMap* other, const Eigen::Affine3d& tf, bool copyCells)
{
    LazyGrid* lz = dynamic_cast<LazyGrid*>(index_);
    int sizeX;
    int sizeY;
    int sizeZ;
    lz->getGridSize(sizeX, sizeY, sizeZ);

    SpatialIndex* other_index = other->getMyIndex();
    LazyGrid* other_lz = dynamic_cast<LazyGrid*>(other_index);
    int other_sizeX;
    int other_sizeY;
    int other_sizeZ;
    other_lz->getGridSize(other_sizeX, other_sizeY, other_sizeZ);

    if (sizeX != other_sizeX and sizeY != other_sizeY and sizeZ != other_sizeZ)
    {
        std::cout << "warning! sizes do not match!" << std::endl;
    }

    ComparisonResult res;

    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            for (int k = 0; k < sizeZ; k++)
            {
                NDTCell* ptr;
                lz->getCellAt(i, j, k, ptr);
                NDTCell* other_ptr;
                other_lz->getCellAt(i, j, k, other_ptr);

                NDTCell* tfd;
                if (other_ptr)
                {
                    Eigen::Vector3d mean = other_ptr->getMean();
                    Eigen::Matrix3d cov = other_ptr->getCov();
                    mean = tf * mean;
                    cov = tf.rotation() * cov * tf.rotation().transpose();
                    tfd = (NDTCell*)other_ptr->clone();
                    tfd->setMean(mean);
                    tfd->setCov(cov);
                }

                if (ptr && not other_ptr)
                {
                    res.other_missing_count_++;
                    ptr->setComparisonStatus(NDTCell::ComparisonState::OTHER_MISSING);
                }
                else if (not ptr && other_ptr)
                {
                    res.this_missing_count_++;
                    if (copyCells)
                    {
                        tfd->setComparisonStatus(NDTCell::ComparisonState::THIS_MISSING);
                        lz->insertCell(*tfd);
                    }
                }
                else if (not ptr && not other_ptr)
                {
                    res.both_missing_count_++;
                }
                else
                {
                    if (ptr->hasGaussian_ && not other_ptr->hasGaussian_)
                    {
                        res.other_no_distribution_count_++;
                        ptr->setComparisonStatus(NDTCell::ComparisonState::OTHER_NO_DISTRIBUTION);
                    }
                    else if (not ptr->hasGaussian_ && other_ptr->hasGaussian_)
                    {
                        res.this_no_distribution_count_++;
                        if (copyCells)
                        {
                            tfd->setComparisonStatus(
                                NDTCell::ComparisonState::THIS_NO_DISTRIBUTION);
                            lz->insertCell(*tfd);
                        }
                    }
                    else if (not ptr->hasGaussian_ && not other_ptr->hasGaussian_)
                    {
                        res.both_no_distribution_count_++;
                        ptr->setComparisonStatus(NDTCell::ComparisonState::BOTH_NO_DISTRIBUTION);
                    }
                    else
                    {
                        res.match_count_++;
                        ptr->setComparisonStatus(NDTCell::ComparisonState::MATCH);
                        double dist = ptr->compare(tfd);
                        dist = fabs(dist);
                        dist = clamp(dist, 0, 1);
                        res.total_distance_ += dist;
                        ptr->setComparisonDistance(dist);

                        if (dist > res.max_distance_)
                        {
                            res.max_distance_ = dist;
                        }
                        if (dist < res.min_distance_)
                        {
                            res.min_distance_ = dist;
                        }
                    }
                }

                if (other_ptr)
                {
                    delete tfd;
                }
            }
        }
    }

    res.avg_distance_ = res.total_distance_ / (double)res.match_count_;

    return res;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

//      ██╗███████╗███████╗
//      ██║██╔════╝██╔════╝
//      ██║█████╗  █████╗
// ██   ██║██╔══╝  ██╔══╝
// ╚█████╔╝██║     ██║
//  ╚════╝ ╚═╝     ╚═╝

/** output methods for saving the map in the jff format
 */
int NDTMap::writeToJFF(const char* filename)
{

    if (filename == NULL)
    {
        // ERR("problem outputing to jff\n");
        return -1;
    }

    FILE* jffout = fopen(filename, "w+b");

    fwrite(_JFFVERSION_, sizeof(char), strlen(_JFFVERSION_), jffout);

    switch (this->getMyIndexInt())
    {
        case 1:
            writeCellVectorJFF(jffout);
            break;
        case 2:
            // writeOctTreeJFF(jffout);
            break;
        case 3:
            writeLazyGridJFF(jffout);
            break;
        default:
            // ERR("unknown index type\n");
            return -1;
    }

    fclose(jffout);

    return 0;
}

int NDTMap::writeCellVectorJFF(FILE* jffout)
{
    int indexType[1] = {1};
    fwrite(indexType, sizeof(int), 1, jffout);

    // ORU_TODO: add CellVector specific stuff

    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell != NULL)
        {
            if (cell->hasGaussian_)
            {
                // ORU_TODO: add index specific content smartly
                if (cell->writeToJFF(jffout) < 0)
                    return -1;
            }
        }
        else
        {
            // do nothing
        }
        it++;
    }

    return 0;
}

int NDTMap::writeOctTreeJFF(FILE* jffout)
{
    int indexType[1] = {2};
    fwrite(indexType, sizeof(int), 1, jffout);

    // ORU_TODO: add OctTree specific stuff

    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        NDTCell* cell = (*it);
        if (cell != NULL)
        {
            if (cell->hasGaussian_)
            {
                // ORU_TODO: add index specific content smartly
                if (cell->writeToJFF(jffout) < 0)
                    return -1;
            }
        }
        else
        {
            // do nothing
        }
        it++;
    }

    return 0;
}

int NDTMap::writeLazyGridJFF(FILE* jffout)
{
    int indexType[1] = {3};
    fwrite(indexType, sizeof(int), 1, jffout);

    // add LazyGrid specific stuff
    double sizeXmeters, sizeYmeters, sizeZmeters;
    double cellSizeX, cellSizeY, cellSizeZ;
    double centerX, centerY, centerZ;
    LazyGrid* ind = dynamic_cast<LazyGrid*>(index_);

    ind->getGridSizeInMeters(sizeXmeters, sizeYmeters, sizeZmeters);
    ind->getCellSize(cellSizeX, cellSizeY, cellSizeZ);
    ind->getCenter(centerX, centerY, centerZ);

    double lazyGridData[9] = {sizeXmeters, sizeYmeters, sizeZmeters, cellSizeX, cellSizeY,
                              cellSizeZ,   centerX,     centerY,     centerZ};

    fwrite(lazyGridData, sizeof(double), 9, jffout);

    fwrite(ind->getProtoType(), sizeof(NDTCell), 1, jffout);

    // loop through all active cells
    typename SpatialIndex::CellVectorItr it = index_->begin();
    while (it != index_->end())
    {
        if ((*it)->writeToJFF(jffout) < 0)
            return -1;
        it++;
    }

    return 0;
}

/** method to load NDT maps from .jff files
USAGE:	create NDTMap with desired index and PointType (index type is
checked, but Point type is NOT checked) via e.g.

lslgeneric::NDTMap<pcl::PointXYZL> nd1(
new lslgeneric::LazyGrid<pcl::PointXYZL>(0.4)); --> (*)

and then call

nd1.loadFromJFF("map0027.jff");

 *) use this constructor so index is not initialized and attributes
 can be set manually
 */
int NDTMap::loadFromJFF(const char* filename)
{

    FILE* jffin;

    if (filename == NULL)
    {
        JFFERR("problem outputing to jff");
    }

    jffin = fopen(filename, "r+b");
    if (jffin == NULL)
    {
        JFFERR("file not found");
        std::cerr << "Failed to open : " << filename << std::endl;
        return -5;
    }

    char versionBuf[16];
    if (fread(&versionBuf, sizeof(char), strlen(_JFFVERSION_), jffin) <= 0)
    {
        JFFERR("reading version failed");
    }
    versionBuf[strlen(_JFFVERSION_)] = '\0';

    int indexType;
    if (fread(&indexType, sizeof(int), 1, jffin) <= 0)
    {
        JFFERR("reading version failed");
    }

    if (indexType != this->getMyIndexInt())
    {
        switch (indexType)
        {
            case 1:
                std::cerr << "Map uses CellVector\n";
                return -1;
                break;
            case 2:
                std::cerr << "Map uses OctTree\n";
                return -2;
                break;
            case 3:
                std::cerr << "Map uses LazyGrid\n";
                return -3;
                break;
        }
    }

    switch (indexType)
    {
        case 1: {
            CellVector* cv = dynamic_cast<CellVector*>(index_);
            if (cv->loadFromJFF(jffin) < 0)
            {
                JFFERR("Error loading CellVector");
            }
            break;
        }
        case 3: {
            std::cerr << "Map uses LazyGrid\n";
            LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
            if (gr->loadFromJFF(jffin) < 0)
            {
                JFFERR("Error loading LazyGrid");
            }
            break;
        }
        default:
            JFFERR("error casting index");
    }

    NDTCell* ptCell = new NDTCell();
    index_->setCellType(ptCell);
    delete ptCell;

    fclose(jffin);

    // std::cout << "map loaded successfully " << versionBuf << std::endl;

    isFirstLoad_ = false;

    return 0;
}

/** method to load NDT maps from .jff files
USAGE:	create NDTMap with desired index and PointType (index type is
checked, but Point type is NOT checked) via e.g.

lslgeneric::NDTMap<pcl::PointXYZL> nd1(
new lslgeneric::LazyGrid<pcl::PointXYZL>(0.4)); --> (*)

and then call

nd1.loadFromJFF("map0027.jff");

 *) use this constructor so index is not initialized and attributes
 can be set manually
 */
int NDTMap::loadFromJFF(FILE* jffin)
{

    char versionBuf[16];
    if (fread(&versionBuf, sizeof(char), strlen(_JFFVERSION_), jffin) <= 0)
    {
        JFFERR("reading version failed");
    }
    versionBuf[strlen(_JFFVERSION_)] = '\0';

    int indexType;
    if (fread(&indexType, sizeof(int), 1, jffin) <= 0)
    {
        JFFERR("reading version failed");
    }

    if (indexType != this->getMyIndexInt())
    {
        switch (indexType)
        {
            case 1:
                std::cerr << "Map uses CellVector\n";
                return -1;
                break;
            case 2:
                std::cerr << "Map uses OctTree\n";
                return -2;
                break;
            case 3:
                std::cerr << "Map uses LazyGrid\n";
                return -3;
                break;
        }
    }

    switch (indexType)
    {
        case 1: {
            CellVector* cv = dynamic_cast<CellVector*>(index_);
            if (cv->loadFromJFF(jffin) < 0)
            {
                JFFERR("Error loading CellVector");
            }
            break;
        }
        case 3: {
            std::cerr << "Map uses LazyGrid\n";
            LazyGrid* gr = dynamic_cast<LazyGrid*>(index_);
            if (gr->loadFromJFF(jffin) < 0)
            {
                JFFERR("Error loading LazyGrid");
            }
            break;
        }
        default:
            JFFERR("error casting index");
    }

    NDTCell* ptCell = new NDTCell();
    index_->setCellType(ptCell);
    delete ptCell;

    // std::cout << "map loaded successfully " << versionBuf << std::endl;

    isFirstLoad_ = false;

    return 0;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

//  ██████╗██╗     ██╗   ██╗███████╗████████╗███████╗██████╗ ██╗███╗   ██╗ ██████╗
// ██╔════╝██║     ██║   ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗██║████╗  ██║██╔════╝
// ██║     ██║     ██║   ██║███████╗   ██║   █████╗  ██████╔╝██║██╔██╗ ██║██║  ███╗
// ██║     ██║     ██║   ██║╚════██║   ██║   ██╔══╝  ██╔══██╗██║██║╚██╗██║██║   ██║
// ╚██████╗███████╗╚██████╔╝███████║   ██║   ███████╗██║  ██║██║██║ ╚████║╚██████╔╝
//  ╚═════╝╚══════╝ ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝ ╚═════╝

void NDTMap::cluster(bool initialize, bool clusterSemiStatic, bool clusterStatic, int max_it)
{
    float sim_threshold = 0.1;
    int min_cluster_num = 10;

    if (initialize)
    {
        // 1. set each cell to be own cluster
        std::cout << "start: # clusters: " << clusters_.size() << std::endl;
        createInitialClusters(clusterSemiStatic, clusterStatic);
        regionGrowingIterationSingle();
        calcNeighbors();
        eraseEmptyClusters();
        std::cout << "initial: # clusters: " << clusters_.size() << std::endl;
    }

    // 2. region growing based on similarity
    int initial_clusters = clusters_.size();
    int it = 0;
    float max_sim = 1;
    int prev_clusters = clusters_.size();
    int joint = 1;
    // while (max_sim > sim_threshold && clusters_.size() > min_cluster_num && it < max_it)
    while (joint > 0 && clusters_.size() > min_cluster_num && it < max_it)
    {
        // max_sim = regionGrowingIteration();
        int joined_est = regionGrowingIterationSingle();
        calcNeighbors();
        // eraseEmptyClusters();
        it++;
        if (it % 100 == 0)
        {
            std::cout << it << ": # clusters: " << clusters_.size() << std::endl;
        }
        int now_clusters = clusters_.size();
        joint = prev_clusters - now_clusters;
        prev_clusters = clusters_.size();
    }

    std::cout << "clustering complete after " << it << " iterations. Initial clusters "
              << initial_clusters << " now " << clusters_.size() << " clusters." << std::endl;
}

bool NDTMap::hasClusters() const
{
    return clusters_.size() > 0;
}

void NDTMap::createInitialClusters(bool clusterSemiStatic, bool clusterStatic)
{
    // clear clusters
    clusters_.clear();

    // 1. set each cell to be own cluster
    std::vector<NDTCell*> cells = getAllCells();
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        NDTCell* cell = cells[i];
        if (cell->hasGaussian_)
        {
            initializeCluster(cell, clusterSemiStatic, clusterStatic);
        }
    }
}

void NDTMap::initializeCluster(NDTCell* cell, bool clusterSemiStatic, bool clusterStatic)
{
    // try to join
    int label = cell->getLabel();
    if (!clusterStatic and ndt_generic::isStatic(label))
    {
        return;
    }
    if (!clusterSemiStatic and ndt_generic::isSemiStatic(label))
    {
        return;
    }

    pcl::PointXYZL cnt = cell->getCenter();
    std::vector<NDTCell*> vec_a = { cell };
    std::vector<NDTCell*> n = getCellsForPoint(cnt, 1, true);
    bool found = false;
    for (unsigned int j = 0; j < n.size(); j++)
    {
        //int l = n[j]->getLabel();
        std::vector<NDTCell*> vec_b = { n[j] };
        double sim = similarity(vec_a, vec_b);
        if (sim > similarity_threshold_)
        {
            int id = n[j]->getClusterId();
            if (id == -1)
            {
                continue;
            }
            cell->setClusterId(id);
            std::vector<NDTCell*> cells = clusters_[id];
            cells.push_back(cell);
            clusters_[id] = cells;
            cluster_updates_[id] = growing_iterations_;
            found = true;
            break;
        }
    }
    if (not found)
    {
        int idx = getNextFreeClusterId();
        cell->setClusterId(idx, 1.0);
        std::vector<NDTCell*> cellVec;
        cellVec.push_back(cell);
        std::vector<int> ns = getNeighborClusters(idx, cellVec);
        clusters_[idx] = cellVec;
        cluster_neighbors_[idx] = ns;
        cluster_updates_[idx] = growing_iterations_;
    }
}

void NDTMap::removeFromCluster(NDTCell* cell)
{
    int id = cell->getClusterId();
    if (id == -1)
    {
        return;
    }
    std::vector<NDTCell*> cluster = clusters_[id];
    auto it = std::find(cluster.begin(), cluster.end(), cell);
    cluster.erase(it);
    cell->setClusterId(-1);
}

std::vector<int> NDTMap::getNeighborClusters(int cluster)
{
    std::vector<NDTCell*> cells = clusters_[cluster];
    return getNeighborClusters(cluster, cells);
}

std::vector<int> NDTMap::getNeighborClusters(int ownId, const std::vector<NDTCell*>& cells)
{
    std::vector<int> neighbors;
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        pcl::PointXYZL cnt = cells[i]->getCenter();
        std::vector<NDTCell*> n = getCellsForPoint(cnt, 1, true);
        for (unsigned int j = 0; j < n.size(); j++)
        {
            int id = n[j]->getClusterId();
            if (std::find(neighbors.begin(), neighbors.end(), id) == neighbors.end() && id != ownId)
            {
                neighbors.push_back(id);
            }
        }
    }
    return neighbors;
}

void NDTMap::calcNeighbors()
{
    cluster_neighbors_.clear();

    std::map<int, std::vector<NDTCell*>>::iterator it;
    for (it = clusters_.begin(); it != clusters_.end(); ++it)
    {
        int id = it->first;
        cluster_neighbors_[id] = getNeighborClusters(id);
    }
}

int NDTMap::regionGrowingIterationSingle()
{
    std::map<int, std::vector<NDTCell*>>::iterator it_a;
    std::vector<std::pair<int, int>> joinlist;
    int joint = 0;
    int idx = 0;
    int sz = clusters_.size();
    bool bk = false;

    int nonei = 0;
    int nosim = 0;
    int tot = 0;

#pragma omp parallel
    {
#pragma omp for
        for (it_a = clusters_.begin(); it_a != clusters_.end(); ++it_a)
        {
            unsigned int size_a = 0;
            unsigned int size_b = -1;
            bool found = false;
            bool inloop = false;
            std::vector<int> neighbors;

            int id_a = it_a->first;
            std::vector<NDTCell*> cells_a = it_a->second;
            unsigned int changed_a;
            try
            {
                neighbors = cluster_neighbors_.at(id_a);
                changed_a = cluster_updates_.at(id_a);
            }
            catch (const std::exception& e)
            {
            }

            for (unsigned int i = 0; i < neighbors.size(); i++)
            {
                int id_b = neighbors[i];
                unsigned int changed_b = cluster_updates_[id_b];
                if (id_a != id_b)
                {
                    inloop = true;
                    bool found = false;
                    double sim;
                    try
                    {
                        sim = cluster_similarities_.at(std::pair<int, int>(id_a, id_b));
                        found = true;
                    }
                    catch (const std::exception& e)
                    {
                        found = false;
                    }

                    if (changed_a >= growing_iterations_ || changed_b >= growing_iterations_ ||
                        !found)
                    {

                        std::vector<NDTCell*> cells_b = clusters_[id_b];
                        size_b = cells_b.size();
                        sim = similarity(cells_a, cells_b);
                        cluster_similarities_[std::pair<int, int>(id_a, id_b)] = sim;
                    }

                    if (sim > similarity_threshold_)
                    {
                        int larger = id_a;
                        int smaller = id_b;
                        if (size_a < size_b)
                        {
                            larger = id_b;
                            smaller = id_a;
                        }
#pragma omp critical
                        {
                            joinlist.push_back(std::pair<int, int>(larger, smaller));
                        }
                        joint++;
                        found = true;
                    }
                }
            }

            if (!inloop)
            {
                nonei++;
            }
            if (!found && inloop)
            {
                nosim++;
            }
            tot++;

            if (bk)
            {
                break;
            }
            idx++;
        }
    }

    std::vector<int> processed;
    processed.resize(joinlist.size());
    for (unsigned int i = 0; i < joinlist.size(); i++)
    {
        std::pair<int, int> ids = joinlist[i];
        int id_a = ids.first;
        int id_b = ids.second;
        if (std::find(processed.begin(), processed.end(), id_b) == processed.end())
        {
            joinClusters(id_a, id_b);
            processed[i] = id_b;
        }
    }

    // std::cout << "nonei " << nonei << " nosim " << nosim << " joint " << joint << " tot " << tot << std::endl;
    growing_iterations_++;
    return joint;
}
double NDTMap::regionGrowingIteration()
{
    std::map<int, std::vector<NDTCell*>>::iterator it_a;
    double max_sim = -1;
    int max_a = -1;
    int max_b = -1;
    int size_a = -1;
    int size_b = -1;
    int idx = 0;
    int sz = clusters_.size();
    bool bk = false;

#pragma omp parallel
    {
#pragma omp for
        for (it_a = clusters_.begin(); it_a != clusters_.end(); ++it_a)
        {
            int id_a = it_a->first;
            std::vector<NDTCell*> cells_a = it_a->second;

            std::vector<int> neighbors = cluster_neighbors_[id_a];
            for (unsigned int i = 0; i < neighbors.size(); i++)
            {
                int id_b = neighbors[i];
                auto search = clusters_.find(id_b);
                if (id_a != id_b && search != clusters_.end())
                {
                    std::vector<NDTCell*> cells_b = clusters_[id_b];
                    double sim = similarity(cells_a, cells_b);

#pragma omp critical
                    {
                        if (sim > max_sim)
                        {
                            max_sim = sim;
                            max_a = id_a;
                            max_b = id_b;
                            size_a = cells_a.size();
                            size_b = cells_b.size();
                            if (max_sim > 0.8 && sz > 1000)
                            {
                                bk = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (bk)
            {
                break;
            }
            idx++;
        }
    }
    if (size_a >= size_b)
    {
        joinClusters(max_a, max_b);
    }
    else
    {
        joinClusters(max_b, max_a);
    }
    return max_sim;
}

Eigen::ArrayXi NDTMap::getCombinedLabelDistribution(const std::vector<NDTCell*>& cells) const
{
    Eigen::ArrayXi total;
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        Eigen::ArrayXi dist = cells[i]->getLabels();
        if (i == 0)
        {
            total = dist;
        }
        else
        {
            total = total + dist;
        }
    }
    return total;
}

int getMajorityLabel(const std::vector<NDTCell*>& cells)
{
    std::map<int, int> labels;
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        int l = cells[i]->getLabel();
        int cnt = labels[l];
        labels[l] = ++cnt;
    }

    std::map<int, int>::iterator it;
    int maxlabel = -1;
    int maxcount = -1;

    for (it = labels.begin(); it != labels.end(); it++)
    {
        int cnt = it->second;
        if (cnt > maxcount)
        {
            maxlabel = it->first;
            maxcount = cnt;
        }
    }
    return maxlabel;
}

double NDTMap::similarity(const std::vector<NDTCell*>& a, const std::vector<NDTCell*>& b) const
{
    //! debug
    int type = 2;
    if (type == 2)
    {
        int l_a = getMajorityLabel(a);
        int l_b = getMajorityLabel(b);
        if (l_a == l_b)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else if (type == 1)
    {
        Eigen::ArrayXi dist_a_unn = getCombinedLabelDistribution(a);
        Eigen::ArrayXi dist_b_unn = getCombinedLabelDistribution(b);
        Eigen::ArrayXd dist_a = NDTCell::normalizeDistribution(dist_a_unn);
        Eigen::ArrayXd dist_b = NDTCell::normalizeDistribution(dist_b_unn);
        double kld = KLDivergence(dist_a, dist_b);
        double sim = weightingFunction(kld);
        return sim;
    }
}

void NDTMap::eraseEmptyClusters()
{
    std::map<int, std::vector<NDTCell*>>::iterator it;
    std::vector<int> removelist;
    for (it = clusters_.begin(); it != clusters_.end(); ++it)
    {
        bool found = false;
        int id = it->first;
        std::vector<NDTCell*> cells = it->second;
        for (unsigned int i = 0; i < cells.size(); i++)
        {
            NDTCell* cell = cells[i];
            if (cell)
            {
                if (cell->getClusterId() == id)
                {
                    found = true;
                    break;
                }
            }
        }
        if (!found)
        {
            removelist.push_back(id);
        }
    }

    for (unsigned int i = 0; i < removelist.size(); i++)
    {
        auto it = clusters_.find(removelist[i]);
        clusters_.erase(it);

        auto it2 = cluster_neighbors_.find(removelist[i]);
        if (it2 != cluster_neighbors_.end())
        {
            cluster_neighbors_.erase(it2);
        }
    }
}

void NDTMap::joinClusters(int a, int b)
{
    std::vector<NDTCell*> cells_a = clusters_[a];
    if (cells_a.size() < 1)
    {
        auto ita = clusters_.find(a);
        clusters_.erase(ita);
        return;
    }

    std::vector<NDTCell*> cells_b = clusters_[b];
    for (unsigned int i = 0; i < cells_b.size(); i++)
    {
        NDTCell* cell = cells_b[i];
        cell->setClusterId(a, 1.0);
        cells_a.push_back(cell);
    }
    clusters_[a] = cells_a;

    std::vector<int> n_a = cluster_neighbors_[a];
    std::vector<int> n_b = cluster_neighbors_[b];
    for (unsigned int i = 0; i < n_b.size(); i++)
    {
        int id = n_b[i];
        if (id != a && id != b && std::find(n_a.begin(), n_a.end(), id) == n_a.end())
        {
            n_a.push_back(id);
        }
    }
    cluster_neighbors_[a] = n_a;

    auto it = clusters_.find(b);
    clusters_.erase(it);

    auto it2 = cluster_neighbors_.find(b);
    cluster_neighbors_.erase(it2);

    cluster_updates_[a] = growing_iterations_;
}

const std::map<int, std::vector<NDTCell*>>& NDTMap::getClusters() const
{
    return clusters_;
}

std::vector<NDTCell*> NDTMap::getCluster(int id, bool& found) const
{
    found = false;
    auto search = clusters_.find(id);
    if (search != clusters_.end())
    {
        found = true;
        return search->second;
    }
    else
    {
        return std::vector<NDTCell*>();
    }
}

void NDTMap::dbgClearUpdates()
{
    std::vector<NDTCell*> cells = getAllCells();
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        cells[i]->viz_updated_ = false;
        cells[i]->viz_accessed_ = false;
        cells[i]->viz_created_ = false;
        cells[i]->viz_occ_update_ = 0;
    }
}

// ███████╗██╗   ██╗██╗██████╗ ███████╗███╗   ██╗ ██████╗███████╗
// ██╔════╝██║   ██║██║██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝
// █████╗  ██║   ██║██║██║  ██║█████╗  ██╔██╗ ██║██║     █████╗
// ██╔══╝  ╚██╗ ██╔╝██║██║  ██║██╔══╝  ██║╚██╗██║██║     ██╔══╝
// ███████╗ ╚████╔╝ ██║██████╔╝███████╗██║ ╚████║╚██████╗███████╗
// ╚══════╝  ╚═══╝  ╚═╝╚═════╝ ╚══════╝╚═╝  ╚═══╝ ╚═════╝╚══════╝

EvidenceContainer::EvidenceContainer()
{
}

void EvidenceContainer::add(NDTCell* cell, const double evidence, const int numPoints)
{
    if (cell)
    {
        Evidence ev(cell, evidence, numPoints);
        int id = ev.clusterId_;
        if (id > 0)
        {
            evidence_.push_back(ev);
            indices_.push_back(id);

            auto search = std::find(allIndices_.begin(), allIndices_.end(), id);
            if (search == allIndices_.end())
            {
                allIndices_.push_back(id);
            }
        }
    }
}

const unsigned int EvidenceContainer::size() const
{
    return evidence_.size();
}

const Evidence& EvidenceContainer::operator[](const int index) const
{
    return evidence_[index];
}

const bool EvidenceContainer::hasBeenUpdated(const NDTCell* cell) const
{
    for (unsigned int i = 0; i < evidence_.size(); i++)
    {
        bool found = evidence_[i].cell_ == cell;
        if (found)
        {
            return true;
        }
    }
    return false;
}

const Evidence& EvidenceContainer::get(const NDTCell* cell, bool& found) const
{
    for (unsigned int i = 0; i < evidence_.size(); i++)
    {
        found = evidence_[i].cell_ == cell;
        if (found)
        {
            return evidence_[i];
        }
    }
    found = false;
}

const std::vector<int> EvidenceContainer::getIndices(const int id) const
{
    std::vector<int> idx;
    for (unsigned int i = 0; i < indices_.size(); i++)
    {
        if (indices_[i] == id)
        {
            idx.push_back(i);
        }
    }
    return idx;
}

Evidence::Evidence(NDTCell* cell, const double evidence, const int numPoints)
    : cell_(cell)
    , evidence_(evidence)
    , numPoints_(numPoints)
{
    clusterId_ = cell_->getClusterId();
    logOddsLikelihood_ = log((evidence_) / (1.0 - evidence_));
    totalLikelihood_ = numPoints_ * logOddsLikelihood_;
}

Evidence::Evidence()
    : cell_(NULL)
    , evidence_(-1.0)
    , numPoints_(-1)
    , clusterId_(-1)
    , logOddsLikelihood_(-1.0)
    , totalLikelihood_(-1.0)
{
}

} // namespace perception_oru
