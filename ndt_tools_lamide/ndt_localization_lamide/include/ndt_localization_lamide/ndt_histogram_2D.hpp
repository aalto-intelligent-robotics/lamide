#ifndef NDT_HISTOGRAM_2D_HPP_
#define NDT_HISTOGRAM_2D_HPP_
#include "ndt_localization_lamide/helper_functions.hpp"
#include <vector>
#include <deque>
#include <ndt_map_lamide/ndt_map.h>
#include <ndt_map_lamide/ndt_cell.h>
#include <math.h> // for PI
#include <algorithm>
#include <numeric>
#include <string>
namespace perception_oru {
  typedef std::vector<std::vector<int> > beam;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*! \breif This struct encapsulates single histogram.
   *
   *
   */

  struct histogram {
    std::vector<std::vector<int> > sphere_classes;
    std::vector<std::vector<int> > planar_classes;
    std::vector<NDTCell*> contributing_cells;
    std::vector<int> distances;
    //std::std::vector<NDTCell*> orientation_contributing_cells;
    ~histogram(){
      //  sphere_classes.clear();
      //  planar_classes.clear();
    }
    histogram(){}
    histogram(int np_, int ns_, int nd_){
      setSize(np_, ns_, nd_);
    }
    void setSize(int np_, int ns_, int nd_){
      planar_classes.resize(np_);
      for(int i = 0; i < np_; ++i)
        planar_classes[i].resize(nd_, 0);

      sphere_classes.resize(ns_);
      for(int i = 0; i < ns_; ++i)
        sphere_classes[i].resize(nd_, 0);

      distances.resize(nd_);

      //std::cout << __LINE__<<":"<<ns_<<":"<<sphere_classes.size()<<":"<<sphere_classes[0].size() << "\n";
    }

    // ~histogram(){
    // 	std::cout <<__LINE__<< " removing histogram\n";
    // 	if(contributing_cells.size() != 0){
    // 		std::cout <<__LINE__<< " removing histogram\n";
    // 		contributing_cells.clear();
    // 		std::cout <<__LINE__<< " removing histogram\n";
    // 		//for(int i = 0; i < planar_classes.size(); ++i)
    // 		//	planar_classes[i].clear();
    // 			std::cout << sphere_classes.size() << "\n";
    // 		sphere_classes.clear();
    // 		std::cout <<__LINE__<< " removing histogram\n";
    // 		//for(int i = 0; i < sphere_classes.size(); ++i)
    // 		//	sphere_classes[i].clear();
    // 		planar_classes.clear();
    // 		std::cout <<__LINE__<< " removing histogram\n";
    // 	}
    // 	std::cout <<__LINE__<< " removing histogram\n";
    // }

    histogram(const histogram& other){
      sphere_classes = other.sphere_classes;
      planar_classes = other.planar_classes;
      contributing_cells = other.contributing_cells;
      distances=other.distances;
    }
  };
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  class histogram_definition {
  protected:

    double te;                                                                                /**< "roundess" tereshold */
    double cs;                                                                                /**< cell size */
    int ns;                                                                                   /**< number of spherical classess */
    int np;                                                                                   /**< number of planar classes */
    int os;                                                                                   /**< number of histogram orienatations */
    double opening;                                                                           /**< sensor opening angle for histogram computation */
    double theta; /**< angularr step sise for histogram bucket */                             // DO WE NEED THIS?
    std::vector<double> begins;                                                /**< distance beginnings of the bins*/
    std::vector<double> ends;                                                /**< distance ends of the bins*/
    Eigen::Vector2d origin;                                                                   /**< position of the cneter of the spcace in global cooridnate frame */
  public:

    //histogram_definition(double te_, int ns_, int np_, std::vector<std::vector<double> >d_steps, double cs_, Eigen::Vector2d origin_, int os_, double opening_);
    histogram_definition(double te_, int ns_, int np_, std::vector<double> begins_, std::vector<double> ends_, double cs_, Eigen::Vector2d origin_, int os_, double opening_);
    double get_te(){ return te; }
    double get_cs(){ return cs; }
    int get_ns(){ return ns; }
    int get_np(){ return np; }
    int get_os(){ return os; }
    double get_opening(){ return opening; }
    //std::vector<std::vector<double> >get_d_steps(){ return d_steps; }
    std::vector<double> get_begins(){ return begins; }
    std::vector<double> get_ends(){ return ends; }
    ~histogram_definition(){};

  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  class histogram_cell : public histogram_definition {
  public:
    std::vector<histogram> histograms;
    std::vector<NDTCell*> contributing_cells;  //cells that were used in creating histogram
    double cell_score; //????? WHY ???? It should be a vector for every orientation
    double cX, cY;                             //location of the cneter of the cell
    double cell_orientation;
    /**
     * Constructor for cells in map.
     * @param te_ roundness threshold
     * @param ns_ number of spherical classes
     * @param np_ number of planar classes.
     * @param d_steps intervals for distance buckets (do not forget about zero at the beginning)
     * @param cs_ cell size in m
     * @param origin_ position of the cell
     * @param os_ orientation statep (default 1)
     * @param opening sensor FOV (if -1 do not use)
     */
    histogram_cell(double te_, int ns_, int np_, std::vector<double> begins_, std::vector<double> ends_ , double cs_, Eigen::Vector2d origin_, double X, double Y, int os_ = 1, double opening_ = -1);

    /**
     * Constructor for single cell from scan.
     * @param map_ observation as NDT map
     * @param te_ roundness threshold
     * @param ns_ number of spherical classes
     * @param np_ number of planar classes.
     * @param d_steps intervals for distance buckets (do not forget about zero at the beginning)
     * @param cs_ cell size in m
     * @param origin_ position of the cell
     * @param os_ orientation statep (default 1)
     * @param opening sensor FOV (if -1 do not use)
     */
    histogram_cell(const std::shared_ptr<NDTMap>& map_, double te_, int ns_, int np_, std::vector<double> begins_, std::vector<double> ends_, double cs_, Eigen::Vector2d origin_, int os_ = 1, double opening = -1);  //constructor for single scan
    double score(histogram_cell outer);
    double scoreNoChange(histogram_cell outer);

    void getSeed(double &X, double &Y, double &th){
      X = cX;
      Y = cY;
      th = cell_orientation;                   //2.0 * M_PI * (double(cell_orientation) / double(os));
    }

    bool compute_histogram();

    ~histogram_cell();
  protected:

    double compare(std::vector<std::vector<int> > F, int n1_F, std::vector<std::vector<int> > G, int n1_G);

    std::vector<int> getDistBin(Eigen::Vector2d center);

    int getOrientationBin(Eigen::Vector2d direction, Eigen::Vector2d center_vector);
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  class histogram_map  {

    double centerX, centerY, centerZ;
    int sizeX, sizeY, sizeZ;
    double cellSize;
    double range;
    double alpha; //???? WHAT IS IT ????
    int os;
    double opening;
    std::vector<beam> beams;
    std::shared_ptr<NDTMap>  map;
    void getIndexForPoint(double X, double Y, int &indX, int &indY);
    std::vector<histogram_cell*> data;

  public:
    std::vector<std::vector<histogram_cell*> > data_grid; //??? DO I NEED THIS ???

    histogram_map(const std::shared_ptr<NDTMap>& map_, double te_, int ns_, int np_,  std::vector<double> begins_, std::vector<double> ends_, double range_, int os_, double opening_ );

    double get_te(){ return data[0]->get_te(); }
    double get_cs(){ return data[0]->get_cs(); }
    int get_ns(){ return data[0]->get_ns(); }
    int get_np(){ return data[0]->get_np(); }
    int get_os(){ return data[0]->get_os(); }

    std::vector<double> get_begins(){ return data[0]->get_begins(); }
    std::vector<double> get_ends(){ return data[0]->get_ends(); }
    void score_map(histogram_cell outer);
    std::vector<std::vector<double> > getHeatMap();
    std::vector<std::vector<double> > getSimilarityMap(double X, double Y,int &indX, int &indY);
    std::vector<std::vector<double> >getOrientationMap();

    std::vector<histogram_cell*> getScoredCells();

    int saveHistogramMap(std::string file_name);
    int loadHistogramMap(std::string file_name);

    void reset();
    ~histogram_map();
    void confusion_matrix();

  protected:
    beam RayTrace( double alpha, double range );
  };

}
#endif
