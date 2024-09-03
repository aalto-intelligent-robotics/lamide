#include <Eigen/Eigenvalues>
#include "ndt_localization_lamide/particle_filter.hpp"
#include "ros/ros.h"

bool myfunction(double i, double j)
{
    return (i > j);
}

perception_oru::particle_filter::particle_filter(
    perception_oru::NDTMap* ndtMap_,
    int particleCount_ /*, init_type initializationType_*/,
    bool be2D_,
    bool forceSIR_,
    double varLimit_,
    int sirCount_)
    : n_effective_under_which_trigger_SIR_update(particleCount_ / 4)
    , _use_new_SIR_update(true)
    , _use_hybrid_strategy(true)
    , _use_euclidean_for_long_distances(true)
    , _use_euclidean_distance(false)
    , _cell_neighbor_to_consider(1)
    , _use_mean_of_scoring(false)
    , be2D(be2D_)
    , particleCount(particleCount_)
    , forceSIR(forceSIR_)
    , varLimit(varLimit_)
    , sirCount(sirCount_)
    , var_for_new_particle(0.5)
    , _motion_model_cov_x(0.5)
    , _motion_model_cov_y(0.7)
    , _motion_model_cov_yaw(0.15)
    , _scaling_factor_gaussian(0.005)
    , ndtMap(new perception_oru::NDTMap(*ndtMap_))
{
    // 	be2D = be2D_;
    //	perception_oru::NDTMap* map_copy = new perception_oru::NDTMap(*ndtMap_);
    //	ndtMap = ndtMap_;
    // initializationType=initializationType_;
    // 	particleCount = particleCount_;
    // 	forceSIR = forceSIR_;
    // 	varLimit = varLimit_;
    // 	sirCount = sirCount_;
    std::cout << "Force SIR " << forceSIR << std::endl;

    // Just making sure the copy went fine.
    //	std::cout << "Initialized cells " << ndtMap_->getAllInitializedCells().size() << " == " <<
    //ndtMap->getAllInitializedCells().size() << std::endl;
    assert(ndtMap_->getAllInitializedCells().size() == ndtMap->getAllInitializedCells().size());
    //	std::cout << "Initialized cells " << ndtMap_->getAllCells().size() << " == " <<
    //ndtMap->getAllCells().size() << std::endl;
    assert(ndtMap_->getAllCells().size() == ndtMap->getAllCells().size());

    double a, b, c;
    ndtMap_->getCellSizeInMeters(a, b, c);
    double a1, b1, c1;
    ndtMap->getCellSizeInMeters(a1, b1, c1);
    //	std::cout << a << " == " << a1 << std::endl;
    //	std::cout << b << " == " << b1 << std::endl;
    //	std::cout << c << " == " << c1 << std::endl;
    assert(a == a1);
    assert(b == b1);
    assert(c == c1);

    ndtMap_->getGridSizeInMeters(a, b, c);
    ndtMap->getGridSizeInMeters(a1, b1, c1);
    //	std::cout << a << " == " << a1 << std::endl;
    //	std::cout << b << " == " << b1 << std::endl;
    //	std::cout << c << " == " << c1 << std::endl;
    assert(a == a1);
    assert(b == b1);
    assert(c == c1);

    weights.resize(particleCount_, 0);

    //	exit(0);
}

int perception_oru::particle_filter::SetupNDTH(double te_,
                                               int ns_,
                                               int np_,
                                               std::vector<double> begins_,
                                               std::vector<double> ends_,
                                               double range_,
                                               int os_,
                                               double opening_)
{

    os = os_;
    // ROS_INFO_STREAM(__FILE__<<":"<<__LINE__<<":"<<os);
    opening = opening_;
    // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    h_map = new histogram_map(
        ndtMap, te_, ns_, np_, begins_, ends_, range_, os_,
        opening_); // does range has to be put explicitly? or can we get it from somwhere
                   // ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
    //	id = 0;
    // ROS_INFO_STREAM(__LINE__);
}

std::vector<std::vector<double>> perception_oru::particle_filter::InitializeNDTH(
    const std::shared_ptr<NDTMap>& localNdtMap)
{
    Eigen::Vector2d orig;
    // ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    double p = 1.0 / particleCount;
    tmp.resize(particleCount, perception_oru::particle(.0, .0, .0, .0, .0, .0, p));
    std::vector<double> probability;
    double t_score;
    orig << 0.0, 0.0;
    histogram_cell local_histogram(localNdtMap, h_map->get_te(), h_map->get_ns(), h_map->get_np(),
                                   h_map->get_begins(), h_map->get_ends(), h_map->get_cs(), orig);
    h_map->score_map(local_histogram);
    // h_map->confusion_matrix();
    std::vector<std::vector<double>> hm = h_map->getHeatMap();
    // ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    std::vector<perception_oru::histogram_cell*> a_cc = h_map->getScoredCells();
    std::vector<perception_oru::histogram_cell*> cc;
    std::vector<double> scores;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // #ifdef VER_1
    // 	//inverting and normalizing weights - version 1
    // 	double score_sum = 0;
    // 	std::cout << "raw_scores=[\n";
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		scores.push_back(a_cc[i]->cell_score);
    // 		score_sum += a_cc[i]->cell_score;
    // 		std::cout << a_cc[i]->cell_score << ";\n";
    // 	}
    // 	std::cout << "];\n probabilities=[\n";
    // 	double mean = score_sum / a_cc.size();
    // 	double probability_sum = 0;
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		probability.push_back(2 * mean - scores[i]);
    // 		probability_sum += probability[i];
    // 	}
    // 	for(int i = 0; i < probability.size(); ++i){
    // 		probability[i] /= probability_sum;
    // 		std::cout << probability[i] << ";\n";
    // 	}
    // 	std::cout << "];\n";
    // 	cc = a_cc;
    // 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // #endif //VER_1
    // #ifdef VER_2
    // 	///inverting and normalizing weights - version 2
    // 	//invertin probabilities
    // 	double score_sum = 0;
    // 	std::cout << "raw_scores=[\n";
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		scores.push_back(a_cc[i]->cell_score);
    // 		score_sum += a_cc[i]->cell_score;
    // 		std::cout << a_cc[i]->cell_score << ";\n";
    // 	}
    // 	//////////////////////////////////////////////////////////////////////////////////////////////
    // 	double median;
    // 	std::vector<double> scores_to_sort = scores;
    //
    // 	size_t size = scores.size();
    // 	sort(scores_to_sort.begin(), scores_to_sort.end());
    // 	if(size  % 2 == 0)
    // 		median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
    // 	else
    // 		median = scores_to_sort[size / 2];
    // 	std::vector<double> filtered_scores;
    // 	std::vector<perception_oru::histogram_cell*> filtered_cc;
    // 	for(int i = 0; i < scores.size(); ++i){
    // 		if(scores[i] < median){
    // 			filtered_scores.push_back(scores[i]);
    // 			filtered_cc.push_back(a_cc[i]);
    // 		}
    // 	}
    // 	a_cc = filtered_cc;
    // 	scores = filtered_scores;
    // 	//////////////////////////////////////////////////////////////////////////////////////////////
    // 	std::cout << "];\n probabilities=[\n";
    // 	double mean = score_sum / a_cc.size();
    // 	double probability_sum = 0;
    // 	int min_id = 0;
    // 	double min_val = std::numeric_limits<double>::max();
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		probability.push_back(mean - scores[i]);
    // 		probability_sum += probability[i];
    // 		if(probability[i] < min_val){
    // 			min_val = probability[i];
    // 			min_id = i;
    // 		}
    // 	}
    // 	probability_sum -= min_val;
    // 	std::vector<double> new_probability;
    // 	int min_id_2 = 0;
    // 	double min_val_2 = std::numeric_limits<double>::max();
    // 	for(int i = 0; i < probability.size(); ++i){
    // 		if(i != min_id){
    // 			new_probability.push_back(probability[i] / probability_sum);
    // 			cc.push_back(a_cc[i]);
    // 			if(new_probability[i] < min_val){
    // 				min_val_2 = new_probability[i];
    // 				min_id_2 = i;
    // 			}
    // 		}
    // 	}
    // 	double new_probability_sum = 0;
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] -= min_val_2;
    // 		new_probability_sum += new_probability[i];
    // 	}
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] /= new_probability_sum;
    // 		std::cout << new_probability[i] << "\n";
    // 	}
    // 	probability = new_probability;
    // //find and remove minimum one
    //
    // 	std::cout << "];\n";
    // #endif //VER_2
    // #ifdef VER_3
    // 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 	///inverting and normalizing weights - version 3
    // 	//invertin probabilities
    // 	double score_sum = 0;
    // 	std::cout << "raw_scores=[\n";
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		scores.push_back(a_cc[i]->cell_score);
    // 		score_sum += a_cc[i]->cell_score;
    // 		std::cout << a_cc[i]->cell_score << ";\n";
    // 	}
    // 	//////////////////////////////////////////////////////////////////////////////////////////////
    // 	double median;
    // 	std::vector<double> scores_to_sort = scores;
    //
    // 	size_t size = scores.size();
    // 	sort(scores_to_sort.begin(), scores_to_sort.end());
    // 	if(size  % 2 == 0)
    // 		median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
    // 	else
    // 		median = scores_to_sort[size / 2];
    // 	std::vector<double> filtered_scores;
    // 	std::vector<perception_oru::histogram_cell*> filtered_cc;
    // 	for(int i = 0; i < scores.size(); ++i){
    // 		if(scores[i] < median){
    // 			filtered_scores.push_back(scores[i]);
    // 			filtered_cc.push_back(a_cc[i]);
    // 		}
    // 	}
    // 	a_cc = filtered_cc;
    // 	scores = filtered_scores;
    // 	//////////////////////////////////////////////////////////////////////////////////////////////
    //
    // 	std::cout << "];\n probabilities=[\n";
    // 	double mean = score_sum / a_cc.size();
    // 	double probability_sum = 0;
    // 	int min_id = 0;
    // 	double min_val = std::numeric_limits<double>::max();
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		probability.push_back(mean - scores[i]);
    // 		probability_sum += probability[i];
    // 		if(probability[i] < min_val){
    // 			min_val = probability[i];
    // 			min_id = i;
    // 		}
    // 	}
    // 	probability_sum -= min_val;
    // 	std::vector<double> new_probability;
    // 	int min_id_2 = 0;
    // 	double min_val_2 = std::numeric_limits<double>::max();
    // 	for(int i = 0; i < probability.size(); ++i){
    // 		if(i != min_id){
    // 			new_probability.push_back(probability[i] / probability_sum);
    // 			cc.push_back(a_cc[i]);
    // 			if(new_probability[i] < min_val){
    // 				min_val_2 = new_probability[i];
    // 				min_id_2 = i;
    // 			}
    // 		}
    // 	}
    // 	double new_probability_sum = 0;
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] -= min_val_2;
    // 		new_probability_sum += new_probability[i];
    // 	}
    // 	for(int i = 0; i < new_probability.size(); ++i)
    // 		new_probability[i] /= new_probability_sum;
    // 	new_probability_sum = 0;
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] = new_probability[i] * new_probability[i];
    // 		new_probability_sum += new_probability[i];
    // 	}
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] /= new_probability_sum;
    // 		std::cout << new_probability[i] << "\n";
    // 	}
    // 	probability = new_probability;
    // 	//find and remove minimum one
    //
    // 	std::cout << "];\n";
    // #endif //VER_3
    // #ifdef VER_4
    // 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 	///inverting and normalizing weights - version 4
    // 	//invertin probabilities
    // 	double score_sum = 0;
    // 	std::cout << "raw_scores=[\n";
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		scores.push_back(a_cc[i]->cell_score);
    // 		score_sum += a_cc[i]->cell_score;
    // 		std::cout << a_cc[i]->cell_score << ";\n";
    // 	}
    //
    // 	//////////////////////////////////////////////////////////////////////////////////////////////
    // 	double median;
    // 	std::vector<double> scores_to_sort = scores;
    // 	std::vector<double> scores_sorted_filtered;
    // 	size_t size = scores.size();
    // 	sort(scores_to_sort.begin(), scores_to_sort.end());
    // 	if(size  % 2 == 0)
    // 		median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
    // 	else
    // 		median = scores_to_sort[size / 2];
    // 	std::vector<double> filtered_scores;
    // 	std::vector<perception_oru::histogram_cell*> filtered_cc;
    // 	for(int i = 0; i < scores.size(); ++i){
    // 		if(scores[i] < median){
    // 			filtered_scores.push_back(scores[i]);
    // 			filtered_cc.push_back(a_cc[i]);
    // 		}
    // 	}
    //
    // 	a_cc = filtered_cc;
    // 	scores = filtered_scores;
    // 	//////////////////////////////////////////////////////////////////////////////////////////////
    //
    // 	std::cout << "];\n probabilities=[\n";
    // 	double mean = score_sum / a_cc.size();
    // 	double probability_sum = 0;
    // 	int min_id = 0;
    // 	double min_val = std::numeric_limits<double>::max();
    // 	for(int i = 0; i < a_cc.size(); ++i){
    // 		probability.push_back(mean - scores[i]);
    // 		probability_sum += probability[i];
    // 		if(probability[i] < min_val){
    // 			min_val = probability[i];
    // 			min_id = i;
    // 		}
    // 	}
    // 	probability_sum -= min_val;
    // 	std::vector<double> new_probability;
    // 	int min_id_2 = 0;
    // 	double min_val_2 = std::numeric_limits<double>::max();
    // 	for(int i = 0; i < probability.size(); ++i){
    // 		if(i != min_id){
    // 			new_probability.push_back(probability[i] / probability_sum);
    // 			cc.push_back(a_cc[i]);
    // 			if(new_probability[i] < min_val){
    // 				min_val_2 = new_probability[i];
    // 				min_id_2 = i;
    // 			}
    // 		}
    // 	}
    // 	double new_probability_sum = 0;
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] -= min_val_2;
    // 		new_probability_sum += new_probability[i];
    // 	}
    // 	for(int i = 0; i < new_probability.size(); ++i)
    // 		new_probability[i] /= new_probability_sum;
    // 	new_probability_sum = 0;
    // ///////////////////////////////////////////////////////////////////////////////////////////
    // 	std::vector<double> probabilities_cumulative;
    // 	std::vector<double> probabilities_sorted = new_probability;
    // 	sort(probabilities_sorted.begin(), probabilities_sorted.end());
    // 	probabilities_cumulative.push_back(probabilities_sorted[0]);
    // 	for(int i = 1; i < probabilities_sorted.size(); ++i)
    // 		probabilities_cumulative.push_back(probabilities_cumulative[i - 1] +
    // probabilities_sorted[i]); 	for(int i = 0; i < probabilities_cumulative.size(); ++i)
    // 		probabilities_cumulative[i] = probabilities_cumulative[i] * probabilities_cumulative[i];
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		for(int j = 0; j < probabilities_sorted.size(); ++j){
    // 			if(new_probability[i] == probabilities_sorted[j]){
    // 				new_probability[i] = probabilities_cumulative[j];
    // 				new_probability_sum += new_probability[i];
    // 				break;
    // 			}
    // 		}
    // 	}
    //
    // ////////////////////////////////////////////////////////////////////////////////////////////
    // 	for(int i = 0; i < new_probability.size(); ++i){
    // 		new_probability[i] /= new_probability_sum;
    // 		std::cout << new_probability[i] << "\n";
    // 	}
    // 	probability = new_probability;
    // 	//find and remove minimum one
    // 	std::cout << "];\n";
    // #endif // VER_4
    //#ifdef VER_5
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // inverting and normalizing weights - version 3
    // invertin probabilities
    double score_sum = 0;
    std::cout << "raw_scores=[\n";
    for (int i = 0; i < a_cc.size(); ++i)
    {
        scores.push_back(a_cc[i]->cell_score);
        // score_sum += a_cc[i]->cell_score;
        std::cout << a_cc[i]->cell_score << ";\n";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    double median;
    std::vector<double> scores_to_sort = scores;

    size_t size = scores.size();
    sort(scores_to_sort.begin(), scores_to_sort.end());
    double fraction = 0.1; // top percent
    int it = floor(double(size) * fraction);
    std::cout << size << "\n";
    std::cout << it << "\n";
    double tr = scores_to_sort[it];
    std::cout << tr << "\n";
    // if(size  % 2 == 0)
    //  median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
    // else
    //  median = scores_to_sort[size / 2];
    std::vector<double> filtered_scores;
    std::vector<perception_oru::histogram_cell*> filtered_cc;
    for (int i = 0; i < scores.size(); ++i)
    {
        if (scores[i] < tr)
        {
            score_sum += scores[i];
            filtered_scores.push_back(scores[i]);
            filtered_cc.push_back(a_cc[i]);
        }
    }
    cc.swap(filtered_cc);
    scores = filtered_scores;
    std::cout << scores.size() << "\n";
    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "];\n probabilities=[\n";
    // double mean = score_sum / a_cc.size();
    // double probability_sum = 0;
    // int min_id = 0;
    // double min_val = std::numeric_limits<double>::max();
    // for(int i = 0; i < filtered_cc.size(); ++i){
    // 	probability.push_back(mean - scores[i]);
    // 	probability_sum += probability[i];
    // 	if(probability[i] < min_val){
    // 		min_val = probability[i];
    // 		min_id = i;
    // 	}
    // }
    // probability_sum -= min_val;
    // std::vector<double> new_probability;
    // int min_id_2 = 0;
    // double min_val_2 = std::numeric_limits<double>::max();
    // for(int i = 0; i < probability.size(); ++i){
    // 	if(i != min_id){
    // 		new_probability.push_back(probability[i] / probability_sum);
    // 		cc.push_back(filtered_cc[i]);
    // 		if(new_probability[i] < min_val){
    // 			min_val_2 = new_probability[i];
    // 			min_id_2 = i;
    // 		}
    // 	}
    // }
    // double new_probability_sum = 0;
    // for(int i = 0; i < new_probability.size(); ++i){
    // 	new_probability[i] -= min_val_2;
    // 	new_probability_sum += new_probability[i];
    // }
    // for(int i = 0; i < new_probability.size(); ++i)
    // 	new_probability[i] /= new_probability_sum;
    //
    // new_probability_sum = 0;
    // for(int i = 0; i < new_probability.size(); ++i){
    // 	new_probability[i] = new_probability[i] * new_probability[i];
    // 	new_probability_sum += new_probability[i];
    // 	std::cout << new_probability[i] << " "<< new_probability_sum << "\n";
    // }
    for (int i = 0; i < scores.size(); ++i)
    {
        probability.push_back(scores[i] / score_sum);
        std::cout << probability[i] << " " << scores[i] << " " << score_sum << "\n";
    }
    // probability = new_probability;
    // find and remove minimum one

    std::cout << "];\n";
    //#endif //VER_5
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // sample from the distribution
    std::mt19937 generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    std::vector<double> rand_points;
    for (int i = 0; i < particleCount; ++i)
        rand_points.push_back(distribution(generator));
    std::sort(rand_points.begin(), rand_points.end());
    int i = 0;
    int j = 0;
    std::vector<int> counter(probability.size(), 0);
    double current_threshold = probability[j];
    while (i < rand_points.size())
    {
        if (rand_points[i] < current_threshold)
        {
            counter[j] = counter[j] + 1;
            i++;
        }
        else
        {
            j++;
            current_threshold += probability[j];
        }
    }
    // std::cout << i<<" "<<counter.size() << "\n";
    // std::cout << "high_variance=[" << "\n";
    // for(int i = 0; i < counter.size(); ++i){
    // 	std::cout << counter[i] << ";\n";
    // 	for(int j = 0; j < counter[i]; ++j){
    // 		double th, x, y;
    // 		cc[i]->getSeed(x, y, th);
    // 		// std::cout << th << "\n";
    // 		std::uniform_real_distribution<double> distribution_x(x - h_map->get_cs() / 2.0, x +
    // h_map->get_cs() / 2.0); 		std::uniform_real_distribution<double> distribution_y(y -
    // h_map->get_cs() / 2.0, y + h_map->get_cs() / 2.0); 		std::uniform_real_distribution<double>
    // distribution_th(th - 5 / 180 * M_PI, th + 5 / 180 * M_PI); 		double r_th =
    // distribution_th(generator); 		double r_x = distribution_x(generator); 		double r_y =
    // distribution_y(generator);
    // 		// std::cout <<x - h_map->get_cs() / 2.0<<" "<< x + h_map->get_cs() / 2.0<<" "<<
    // h_map->get_cs() << "\n";
    // 		//std::cout << "p=" << r_x << " " << r_y << " " << r_th << "\n";
    // 		to2PI(r_th);
    // 		//particleCloud.emplace_back(0.0, 0.0, r_th, r_x, r_y, 0.0);
    // 	}
    // }
    // std::cout << "];" << "\n";
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    //  std::cout << cc[i]->cell_score << " " << probability[i] << "\n";
    std::uniform_real_distribution<double> distribution_l(0.0, 1.0 / (double)particleCount);
    double r = distribution_l(generator);
    // //std::cout << r << "\n";
    double c = probability[0];
    // int
    i = 0;
    // //	std::default_random_engine generator;
    // std::cout << "low_variance=[" << "\n";
    for (int m = 0; m < particleCount; m++)
    {
        double U = r + double(m) / double(particleCount);
        while (U > c)
        {
            if (i > probability.size())
                i = 0;
            else
                i++;
            c = c + probability[i];
        }
        //  //draw a point from i cell and add to set of partciles
        double th, x, y;
        // std::cout << i << ";\n";
        cc[i]->getSeed(x, y, th);
        //  // std::cout << th << "\n";
        std::uniform_real_distribution<double> distribution_x(x - h_map->get_cs() / 2.0,
                                                              x + h_map->get_cs() / 2.0);
        std::uniform_real_distribution<double> distribution_y(y - h_map->get_cs() / 2.0,
                                                              y + h_map->get_cs() / 2.0);
        std::uniform_real_distribution<double> distribution_th(
            th - ((M_PI / 180.0) * (360.0 / double(h_map->get_os()))) / 2.0,
            th + ((M_PI / 180.0) * (360.0 / double(h_map->get_os()))) / 2.0);
        double r_th = distribution_th(generator);
        double r_x = distribution_x(generator);
        double r_y = distribution_y(generator);
        // std::cout <<x - h_map->get_cs() / 2.0<<" "<< x + h_map->get_cs() / 2.0<<" "<<
        // h_map->get_cs() << "\n"; std::cout<<"p=" << r_x << " " << r_y << " " << r_th << "\n";
        to2PI(th);
        // particleCloud.emplace_back(0, 0, r_th, r_x, r_y,
        // 0,1.0/double(particleCount),1.0/double(particleCount));
        particleCloud.emplace_back(0, 0, r_th, r_x, r_y, 0, particleCount);
    }
    // std::cout << "];\n";
    // ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    // for (int i = 0; i < cc.size(); ++i)
    // {
    // 	ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    // 	if(cc[i]!=NULL)
    // 		cc[i]=NULL;
    // }
    // cc.clear();
    // ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    // for (int i = 0; i < a_cc.size(); ++i)
    // {
    // 	ROS_INFO_STREAM(__FILE__ << ":" << __LINE__<<":"<<i);
    // 	if(a_cc[i]!=NULL){
    // 		a_cc[i]=NULL;
    // 	}
    // }
    // a_cc.clear();
    // for (int i = 0; i < filtered_cc.size(); ++i)
    // {
    // 	ROS_INFO_STREAM(__FILE__ << ":" << __LINE__<<":"<<i);
    // 	if(filtered_cc[i]!=NULL)
    // 		filtered_cc[i]=NULL;
    // }
    // filtered_cc.clear();
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    // h_map->reset();
    // ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
    return hm;
}

void perception_oru::particle_filter::Reset()
{
    //	tmp.clear();
    // 	delete ndt_ISSMap;
    particleCloud.clear();
}

void perception_oru::particle_filter::InitializeNormalConstantAngle(double x,
                                                                    double y,
                                                                    double th,
                                                                    double var)
{

    if (particleCloud.size() > 0)
    {
        particleCloud.clear();
        //		tmp.clear();
    }
    m_pose << x, y, th;
    //	tmp.resize(particleCount);
    std::default_random_engine generator;

    for (int parNo = 0; parNo < particleCount; parNo++)
    {
        std::normal_distribution<double> distribution_x(x, var);
        std::normal_distribution<double> distribution_y(y, var);
        //		std::normal_distribution<double> distribution_t(th, th * var);
        double xx, yy, yawyaw;
        yawyaw = th;
        xx = distribution_x(generator);
        yy = distribution_y(generator);
        // 		std::cout << "Particule at : " << xx << " " << yy << " " << yawyaw << std::endl;
        particleCloud.emplace_back(0.0, 0.0, yawyaw, xx, yy, 0.0, particleCount);
    }
    //	tmp = particleCloud;
}

void perception_oru::particle_filter::InitializeNormal(double x, double y, double th, double var)
{

    if (particleCloud.size() > 0)
    {
        particleCloud.clear();
        //		tmp.clear();
    }
    m_pose << x, y, th;
    //	tmp.resize(particleCount);
    std::default_random_engine generator;

    for (int parNo = 0; parNo < particleCount; parNo++)
    {
        std::normal_distribution<double> distribution_x(x, var);
        std::normal_distribution<double> distribution_y(y, var);
        std::normal_distribution<double> distribution_t(th, th * var);
        double xx, yy, yawyaw;
        yawyaw = distribution_t(generator);
        xx = distribution_x(generator);
        yy = distribution_y(generator);
        // 		std::cout << "Particule at : " << xx << " " << yy << " " << yawyaw << std::endl;
        particleCloud.emplace_back(0.0, 0.0, yawyaw, xx, yy, 0.0, particleCount);
    }
    //	tmp = particleCloud;
}

void perception_oru::particle_filter::InitializeNormal(
    double x, double y, double z, double th, double var)
{

    if (particleCloud.size() > 0)
    {
        particleCloud.clear();
        //		tmp.clear();
    }
    m_pose << x, y, th;
    //	tmp.resize(particleCount);
    std::default_random_engine generator;

    for (int parNo = 0; parNo < particleCount; parNo++)
    {
        std::normal_distribution<double> distribution_x(x, var);
        std::normal_distribution<double> distribution_y(y, var);
        // 		std::normal_distribution<double> distribution_z(z, var);
        std::normal_distribution<double> distribution_t(th, th * var);
        double xx, yy, zz, yawyaw;
        yawyaw = distribution_t(generator);
        xx = distribution_x(generator);
        yy = distribution_y(generator);
        // 		zz = distribution_z(generator);
        // 		std::cout << "Particule at : " << xx << " " << yy << " " << yawyaw << std::endl;
        particleCloud.emplace_back(0.0, 0.0, yawyaw, xx, yy, z, particleCount);
    }
    //	tmp = particleCloud;
}

void perception_oru::particle_filter::InitializeNormal(double x, double y, double var)
{

    if (particleCloud.size() > 0)
    {
        particleCloud.clear();
        //		tmp.clear();
    }
    m_pose << x, y, 0;
    //	tmp.resize(particleCount, );
    std::default_random_engine generator;
    for (int parNo = 0; parNo < particleCount; parNo++)
    {
        std::normal_distribution<double> distribution_x(x, var);
        std::normal_distribution<double> distribution_y(y, var);
        std::uniform_real_distribution<double> distribution_t(0, 2 * 3.1415);
        particleCloud.emplace_back(0.0, 0.0, distribution_t(generator), distribution_x(generator),
                                   distribution_y(generator), 0.0, particleCount);
    }
    //	tmp = particleCloud;
}

Eigen::Vector3d perception_oru::particle_filter::GetMeanPose2D()
{
    double sumX = 0, sumY = 0;
    Eigen::Vector3d pos;
    double sumW = 0;
    double ax = 0, ay = 0;

    for (int i = 0; i < particleCloud.size(); i++)
    {
        double x, y, z, r, p, t;
        particleCloud[i].GetXYZ(x, y, z);
        particleCloud[i].GetRPY(r, p, t);
        sumX += particleCloud[i].GetProbability() * x;
        sumY += particleCloud[i].GetProbability() * y;
        ax += particleCloud[i].GetProbability() * cos(t);
        ay += particleCloud[i].GetProbability() * sin(t);
        sumW += particleCloud[i].GetProbability();
    }
    pos << sumX, sumY, atan2(ay, ax);
    return pos;
}

void perception_oru::particle_filter::GetPoseMeanAndVariance2D(Eigen::Vector3d& mean,
                                                               Eigen::Matrix3d& cov)
{
    double sumX = 0, sumY = 0;
    double sumW = 0;
    double ax = 0, ay = 0;

    // 	std::cout << "Particle size " << particleCloud.size() << std::endl;
    assert(particleCloud.size() > 0);
    for (int i = 0; i < particleCloud.size(); i++)
    {
        // 		std::cout << "sumX : " << sumX << " with proba " <<
        // particleCloud[i].GetProbability();
        double x, y, z, r, p, t;
        particleCloud[i].GetXYZ(x, y, z);
        // 		std::cout << " and pose " << x << " " << y << " " << z << std::endl;
        particleCloud[i].GetRPY(r, p, t);
        sumX += particleCloud[i].GetProbability() * x;
        sumY += particleCloud[i].GetProbability() * y;
        ax += particleCloud[i].GetProbability() * cos(t);
        ay += particleCloud[i].GetProbability() * sin(t);
        sumW += particleCloud[i].GetProbability();
    }
    mean << sumX, sumY, atan2(ay, ax);

    double xx = 0, yy = 0, xy = 0, aax = 0, aay = 0;
    double cax = cos(atan2(ay, ax));
    double say = sin(atan2(ay, ax));
    double w2 = 0;

    for (int i = 0; i < particleCloud.size(); i++)
    {
        double x, y, z, r, p, t;
        particleCloud[i].GetXYZ(x, y, z);
        particleCloud[i].GetRPY(r, p, t);
        xx += particleCloud[i].GetProbability() * (x - sumX) * (x - sumX);
        yy += particleCloud[i].GetProbability() * (y - sumY) * (y - sumY);
        xy += particleCloud[i].GetProbability() * (x - sumX) * (y - sumY);
        aax += particleCloud[i].GetProbability() * (cos(t) - cax) * (cos(t) - cax);
        aay += particleCloud[i].GetProbability() * (sin(t) - say) * (sin(t) - say);
        w2 += particleCloud[i].GetProbability() * particleCloud[i].GetProbability();
    }

    if (w2 == 1.0)
    {
        fprintf(stderr, "CParticleFilter::getDistributionVariances -- w2=%lf Should not happen!\n",
                w2);
        w2 = 0.99;
    }
    double wc = 1.0 / (1.0 - w2);
    cov << wc * xx, wc * xy, 0, wc * xy, wc * yy, 0, 0, 0, atan2(wc * aay, wc * aax);

    //	std::cout << wc << std::endl;
    //	std::cout << xx << " " << xy << " " << yy << std::endl;
    //	std::cout << cov << std::endl;
    //	assert( cov(0,0) >= 0);
    //	assert( cov(0,1) >= 0);
    //	assert( cov(1,0) >= 0);
    //	assert( cov(1,1) >= 0);
    //	assert( cov(2,2) >= 0);
}

void perception_oru::particle_filter::InitializeUniformMap()
{
    int pCount_ = particleCount;

    std::vector<perception_oru::NDTCell*> allCells = ndtMap->getAllInitializedCells();
    std::vector<perception_oru::NDTCell*> cells;
    for (int cInd = 0; cInd < allCells.size(); cInd++)
    {
        if (!allCells[cInd]->hasGaussian_)
        {
            cells.push_back(allCells[cInd]);
        }
    }
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution_c(0, cells.size() - 1);
    //	tmp.resize(pCount_);
    for (int parNo = 0; parNo < pCount_; parNo++)
    {
        int cellId = distribution_c(generator);
        double cx, cy, cz, sx, sy, sz;
        if (be2D)
        {
            cells[cellId]->getCenter(cx, cy, cz);
            cells[cellId]->getDimensions(sx, sy, sz);
            std::uniform_real_distribution<double> distribution_x(cx - sx / 2.0, cx + sx / 2.0);
            std::uniform_real_distribution<double> distribution_y(cy - sy / 2.0, cy + sy / 2.0);
            std::uniform_real_distribution<double> distribution_t(0.0, 2 * M_PI);
            particleCloud.emplace_back(0.0, 0.0, distribution_t(generator),
                                       distribution_x(generator), distribution_y(generator), 0.0,
                                       particleCount);
        }
        // here will go 3d distibution
    }
    //	tmp = particleCloud;
}

void perception_oru::particle_filter::InitializeFilter()
{
    // switch(initializationType){
    // case uniform_map:
    InitializeUniformMap();
    // break;
    //  default:3
    //}
}

void perception_oru::particle_filter::UpdateAndPredict(Eigen::Affine3d tMotion,
                                                       perception_oru::NDTMap ndtLocalMap_)
{
    Eigen::Vector3d tr = tMotion.translation();
    Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);

    if (tr[0] != 0.0 && tr[1] != 0.0 && rot[2] != 0)
    {
        Predict2D(tr[0], tr[1], rot[2], tr[0] * 0.03 + 0.005, tr[1] * 0.03 + 0.005,
                  rot[2] * 0.08 + 0.01);
#pragma omp parallel for
        for (int i = 0; i < particleCloud.size(); i++)
        {
            Eigen::Affine3d T = particleCloud[i].GetAsAffine();
            std::vector<perception_oru::NDTCell*> ndts;
            ndts = ndtLocalMap_.pseudoTransformNDT(T);
            double score = 1;
            if (ndts.size() == 0)
                fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
            for (int n = 0; n < ndts.size(); n++)
            {
                Eigen::Vector3d m = ndts[n]->getMean();
                perception_oru::NDTCell* cell;
                pcl::PointXYZL p;
                p.x = m[0];
                p.y = m[1];
                p.z = m[2];
                if (ndtMap->getCellForPoint(p, cell))
                {
                    if (cell == NULL)
                        continue;
                    if (cell->hasGaussian_)
                    {
                        Eigen::Matrix3d covCombined = cell->getCov() + ndts[n]->getCov();
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
                    else
                    {
                    }
                }
            }
            particleCloud[i].SetLikelihood(score);
            for (unsigned int j = 0; j < ndts.size(); j++)
                delete ndts[j];
        }
        Normalize();
        if (forceSIR)
            SIRUpdate();
        else
        {
            double varP = 0;
            for (int i = 0; i < particleCloud.size(); i++)
                varP += (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size())) *
                        (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
            varP /= double(particleCloud.size());
            varP = sqrt(varP);
            if (varP > varLimit || sinceSIR > sirCount)
            {
                // fprintf(stderr,"-SIR- ");
                sinceSIR = 0;
                SIRUpdate();
            }
            else
                sinceSIR++;
        }
    }
}
void perception_oru::particle_filter::UpdateAndPredict(Eigen::Affine3d tMotion,
                                                       perception_oru::NDTMap* ndtLocalMap_)
{

    ROS_DEBUG_STREAM("Update and predict");

    Eigen::Vector3d tr = tMotion.translation();
    Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);

    if (tr[0] != 0.0 && tr[1] != 0.0 && rot[2] != 0)
    {
        Predict2D(tr[0], tr[1], rot[2], tr[0] * 0.1 + 0.005, tr[1] * 0.1 + 0.005,
                  rot[2] * 0.1 + 0.001);
#pragma omp parallel for
        for (int i = 0; i < particleCloud.size(); i++)
        {
            Eigen::Affine3d T = particleCloud[i].GetAsAffine();
            std::vector<perception_oru::NDTCell*> ndts;
            ndts = ndtLocalMap_->pseudoTransformNDT(T);
            double score = 1;
            if (ndts.size() == 0)
                fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
            for (int n = 0; n < ndts.size(); n++)
            {
                Eigen::Vector3d m = ndts[n]->getMean();
                perception_oru::NDTCell* cell;
                // pcl::PointXYZL p;
                // p.x = m[0];p.y=m[1];p.z=m[2];
                pcl::PointXYZL p;
                p.x = m[0];
                p.y = m[1];
                p.z = m[2];

                ROS_DEBUG_STREAM("Gaussian cells " << ndtMap->getAllCellsShared().size());

                if (ndtMap->getCellForPoint(p, cell))
                {
                    if (cell == NULL)
                        continue;
                    ROS_DEBUG_STREAM("Cell : " << cell);
                    if (cell->hasGaussian_)
                    {

                        ROS_DEBUG_STREAM("GaussianS");

                        Eigen::Matrix3d covCombined = cell->getCov() + ndts[n]->getCov();
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
                    else
                    {
                    }
                }
            }
            particleCloud[i].SetLikelihood(score);
            for (unsigned int j = 0; j < ndts.size(); j++)
                delete ndts[j];
        }
        Normalize();
        if (forceSIR)
            SIRUpdate();
        else
        {
            double varP = 0;
            for (int i = 0; i < particleCloud.size(); i++)
                varP += (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size())) *
                        (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
            varP /= double(particleCloud.size());
            varP = sqrt(varP);
            if (varP > varLimit || sinceSIR > sirCount)
            {
                // fprintf(stderr,"-SIR- ");
                sinceSIR = 0;
                SIRUpdate();
            }
            else
                sinceSIR++;
        }
    }
}

Eigen::Affine3d perception_oru::particle_filter::UpdateAndPredictEff(
    Eigen::Affine3d tMotion,
    const pcl::PointCloud<pcl::PointXYZL>& cloud,
    const Eigen::Affine3d& sensorpose,
    double subsample_level,
    double z_cut)
{

    pcl::PointCloud<pcl::PointXYZL> transformed_cloud;
    pcl::transformPointCloud(cloud, transformed_cloud, sensorpose);
    UpdateAndPredictEff(tMotion, transformed_cloud, subsample_level, z_cut);
}

Eigen::Affine3d perception_oru::particle_filter::UpdateAndPredictEff(
    Eigen::Affine3d tMotion,
    const pcl::PointCloud<pcl::PointXYZL>& transformed_cloud,
    double subsample_level,
    double z_cut)
{
    ROS_INFO_STREAM("I am updating and predicting");

    double cellx, celly, cellz;
    bool out = getReferenceMapCellSizes(cellx, celly, cellz);

    ROS_DEBUG_STREAM("Got size" << out);
    ROS_DEBUG_STREAM("Cells size " << cellx << " " << celly << " " << cellz
                                   << " only work with cell of same size on x and y");

    assert(cellx == celly);
    assert(celly == cellz);
    if (cellx != celly)
        exit(0);
    if (celly != cellz)
        exit(0);

    // Cellx/celly/cellz represent the resolution of the internal map of the localization if they
    // are all equal.

    perception_oru::NDTMap* localMap =
        new perception_oru::NDTMap(new perception_oru::LazyGrid(cellx));
    localMap->loadPointCloud(transformed_cloud);
    localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    UpdateAndPredictEff(tMotion, localMap, subsample_level, z_cut);

    delete localMap;
}

Eigen::Affine3d perception_oru::particle_filter::UpdateAndPredictEff(
    Eigen::Affine3d tMotion,
    perception_oru::NDTMap* ndtLocalMap_,
    double subsample_level,
    double z_cut)
{ // you may add z cut here if necessarry

    ROS_INFO_STREAM("I am updating and predicting");

    Eigen::Affine3d ret;
    bool ret_done = false;
    bool no_mov = true;

    // 	std::cout << "GO" << std::endl;

    if (subsample_level < 0 || subsample_level > 1)
        subsample_level = 1;

    Eigen::Vector3d tr = tMotion.translation();
    Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);

    // 	std::cout << "Motion " << tMotion.matrix() << " trans " << tr << " rot " << rot <<
    // std::endl; 	std::cout << tr[0] << " != " << 0.0  << " && " << tr[1] << " != " << 0.0 << " && "
    // << rot[2] << " != " << 0 << std::endl;

    if (tr[0] != 0.0 || tr[1] != 0.0 || rot[2] != 0)
    {
        no_mov = false;
        int all_cells;
        int subsample_cells;
        int used_cells = 0;
        Predict2D(tr[0], tr[1], rot[2], fabs(tr[0]) * _motion_model_cov_x + 0.001,
                  fabs(tr[1]) * _motion_model_cov_y + 0.001,
                  fabs(rot[2]) * _motion_model_cov_yaw +
                      0.001); // this line!!!!!!!!!!!!!!!!!!!!!!!!!
        // Predict2D(tr[0], tr[1], rot[2], fabs(tr[0]) * 0.25 + 0.001, fabs(tr[1]) * 0.25 + 0.001,
        // fabs(rot[2]) * 0.15 + 0.001); //this line!!!!!!!!!!!!!!!!!!!!!!!!!
        std::vector<perception_oru::NDTCell*> ndts0 = ndtLocalMap_->getAllCells();
        std::vector<perception_oru::NDTCell*> ndts;
        if (subsample_level != 1)
        {
            srand(time(NULL));
            for (int i = 0; i < ndts0.size(); ++i)
            {
                double p = ((double)rand()) / RAND_MAX;

                if (p < subsample_level && ndts0[i]->getMean()[2] > z_cut)
                    ndts.push_back(ndts0[i]);
                //				else
                //					delete ndts0[i];
            }
            // exit(0);
        }
        else
        {
            ndts = ndts0;
            ROS_DEBUG_STREAM("No sub sample and size of cells " << ndts.size());
        }

        assert(ndts0.size() > 0);
        assert(ndtMap->getAllCellsShared().size() > 0);
        // 		std::cout << "NDT CELL USED : " << ndts.size() << " gaussian in map " <<
        // ndtMap->getAllCellsShared().size() << std::endl;

        // 		auto cells = ndtMap->getAllCellsShared();
        //
        // 		for(auto it = cells.begin() ; it != cells.end() ; ++it){
        // 			Eigen::Vector3d mm = (*it)->getMean();
        // 			//if(m[2] < zfilt_min) continue;
        // 			pcl::PointXYZL pm;
        // 			pm.x = mm[0]; pm.y = mm[1]; pm.z = mm[2];
        // // 			std::cout << " gaussian are at point : " << pm.x << " " << pm.y << " " << pm.z <<
        // std::endl;
        // 		}

        //#pragma omp parallel for

        int all_cell_gaussians = 0;

        for (int i = 0; i < particleCloud.size(); i++)
        {
            Eigen::Affine3d T = particleCloud[i].GetAsAffine();
            //			if(ret_done == false){
            ret = T;
            //				std::cout << "PARTICULE" << std::endl;
            //				ret_done = true;
            //			}

            // std::vector<perception_oru::NDTCell*> ndts;
            // ndts = ndtLocalMap_->pseudoTransformNDT(T);

            // NOT SURE WHY THIS WAS TO 1... :/ Keep the particle if no mathcing is found :S ?
            //			double score = 1;
            double score = 1;
            double score_old = 1;
            double score_tmp_nul_vec = 1;

            if (ndts.size() == 0)
                fprintf(stderr, "ERROR no gaussians in measurement!!!\n");

            int gaussian_cells = 0;

            for (int n = 0; n < ndts.size(); n++)
            {
                Eigen::Vector3d m = T * ndts[n]->getMean();
                // 				std::cout << " at point : " << T.matrix()  << " * " <<ndts[n]->getMean() <<
                // std::endl;
                // if(m[2] < zfilt_min) continue;
                pcl::PointXYZL p;
                p.x = m[0];
                p.y = m[1];
                p.z = m[2];
                // 				std::cout << " at point : " << p.x << " " << p.y << " " << p.z <<
                // std::endl; 				if(ndtMap->getCellAtPoint(p, cell)){

                std::vector<perception_oru::NDTCell*> cells;

                perception_oru::NDTCell* cell_for_test = NULL;

                // If we do not use the mean of the scores, then we only use the closest cell.
                double score_tmp = 0;

                if (_use_hybrid_strategy)
                {

                    //					std::cout << "Hybrid Strategy for MCL -..............- experimental"
                    //<< std::endl;
                    double score_res = 0;
                    std::tie(score_res, gaussian_cells) = hybridScoring(*ndts[n], m, T);
                    score = score + score_res;

                    //					std::cout << "Update to score " << score_res << " and score " <<
                    //score << std::endl;
                }
                else
                {
                    // Only use one cell
                    if (_use_mean_of_scoring == false)
                    {
                        //					std::cout << "Single cell selection " << std::endl;
                        perception_oru::NDTCell* cell_closest;
                        ndtMap->getCellAtPoint(p, cell_closest);
                        cells.push_back(cell_closest);
                        assert(cells.size() == 1);
                    }
                    // Otherwise we get all the neighboring cells.
                    else
                    {
                        //					std::cout << "Multiple cell selection " <<
                        //_cell_neighbor_to_consider << std::endl;
                        cells = ndtMap->getCellsForPoint(p, _cell_neighbor_to_consider, true);

                        ndtMap->getCellAtPoint(p, cell_for_test);
                        //					cells.push_back(cell_closest);
                        //					assert(cells.size() == 1);
                        // Might be more than zero since we are checking for gaussians already :)
                        //					assert(cells.size() > 0);
                    }

                    for (auto cell : cells)
                    {
                        // 					std::cout << "Cell is : " << cell << " ";
                        if (cell != NULL)
                        {
                            // 					    std::cout << " Not NULL ";
                            // 						if(cell->getClass()!=perception_oru::NDTCell::HORIZONTAL){

                            // 							std::cout << "Cell is : " << cell->getCenter().x << " " <<
                            // cell->getCenter().y << " " << cell->getCenter().z; 					std::cout << "Good
                            // class" << std::endl;
                            if (cell->hasGaussian_)
                            {
                                gaussian_cells++;

                                if (!_use_euclidean_distance)
                                {

                                    //		 						std::cout << " Cell has gaussian
                                    //";
                                    Eigen::Matrix3d map_cov = cell->getCov();
                                    // cell->blurCov( 10 );

                                    // Eigen::Matrix3d covCombined = cell->getCov() + T.rotation() *
                                    // ndts[n]->getCov() * T.rotation().transpose();
                                    //								Eigen::Matrix3d covCombined = map_cov +
                                    //T.rotation() * ndts[n]->getCov() * T.rotation().transpose();
                                    Eigen::Matrix3d covCombined =
                                        map_cov +
                                        T.rotation().transpose() * ndts[n]->getCov() * T.rotation();
                                    Eigen::Matrix3d icov;
                                    bool exists;
                                    double det = 0;

                                    // I guess this step takes forever... if too much cells are
                                    // here... :(
                                    covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                                    if (exists)
                                    {
                                        double l =
                                            (cell->getMean() - m).dot(icov * (cell->getMean() - m));

                                        if (_use_euclidean_for_long_distances)
                                        {
                                            // We use euclidean if probabilistic is too small !
                                            double distance_euclid = (m - cell->getMean()).norm();
                                            if (l <= distance_euclid)
                                            {
                                                score_tmp +=
                                                    0.1 + 0.9 * exp(-_scaling_factor_gaussian *
                                                                    distance_euclid / 2.0);
                                            }
                                            else
                                            {
                                                score_tmp +=
                                                    0.1 + 0.9 * exp(-_scaling_factor_gaussian *
                                                                    distance_euclid / 2.0);
                                            }
                                        }
                                        else
                                        {
                                            score_tmp +=
                                                0.1 +
                                                0.9 *
                                                    exp(-_scaling_factor_gaussian * l /
                                                        2.0); // l is distance between means. 0.005
                                                              // is scaling down that distance
                                                              // simulating "bigger" Gaussians.
                                            // 										double score_textbook += exp( -l /
                                            // 2);
                                        }

                                        Eigen::Vector3d nul_vec = Eigen::Vector3d::Zero();
                                        // 									if(l * 0 == 0){
                                        double l_null = (nul_vec - m).dot(icov * (nul_vec - m));
                                        // 									std::cout << " m " << m(2) << " and " <<
                                        // cell->getMean()(2) << std::endl;
                                        score_tmp_nul_vec +=
                                            0.1 +
                                            0.9 * exp(-_scaling_factor_gaussian * l_null / 2.0);
                                        score_old +=
                                            0.1 + 0.9 * exp(-_scaling_factor_gaussian * l / 2.0);

                                        //									std::cout << "DIFF : "
                                        //<<score_tmp_nul_vec << " and now " << score_tmp << " and
                                        //score old " << score_old << std::endl;
                                        // 									}
                                        // 									else{
                                        // 										std::cout << "how is it possible ? " << l << "
                                        // and " << l * 0 << std::endl; 										exit(0);
                                        // 									}
                                    }
                                    else
                                    {
                                        // 									std::cout << "Doesn't exist " <<
                                        // std::endl;
                                    }
                                }
                                else
                                {
                                    // Using the euclidean distance instead

                                    double distance = (m - cell->getMean()).norm();
                                    score_tmp +=
                                        0.1 + 0.9 * exp(-_scaling_factor_gaussian * distance / 2.0);
                                    //								std::cout << "Using the mean : " <<
                                    //cell->getMean() << " and m " << m << " Distance euclidean " <<
                                    //distance << " thus the score_tmp is " << score_tmp << "we
                                    //added " << 0.1 + 0.9 * exp(-_scaling_factor_gaussian *
                                    //distance / 2.0) << std::endl;
                                }
                            }
                            else
                            {
                                //		 						std::cout << " No gaussians " <<
                                //std::endl;
                            }
                            // 						}else{
                            // 							std::cout << "Cell is not horizontal: " << cell->getClass() <<
                            // " while h is " << perception_oru::NDTCell::HORIZONTAL;
                            // 						}
                        }
                        else
                        {
                            // 						std::cout << " Cell is NULL ";
                        }
                    }

                    // Calculate the mean of all likelihood for the Gaussians and add it to the
                    // final score.
                    if (cells.size() > 0 && _use_mean_of_scoring)
                    {
                        //					std::cout << "Score " << score_tmp << " for cell size " <<
                        //cells.size() << std::endl; 					assert(cells.size() > 0);
                        score_tmp = score_tmp / cells.size();

                        //					//TESTING
                        //					if(cell_for_test != NULL){
                        //						if(cell_for_test->hasGaussian_){
                        //							double score_for_test = 0;
                        //							if (_use_euclidean_distance) {
                        //								double distance_for_test = (m -
                        //cell_for_test->getMean()).norm();
                        ////								std::cout << "Euclidean dist " << distance_for_test <<
                        ///std::endl;
                        //								score_for_test = 0.1 + 0.9 * exp(-_scaling_factor_gaussian
                        //* distance_for_test / 2.0);
                        //							}
                        ////							std::cout << "Single cell score " << score_for_test << " mean
                        ///cell score " << score_tmp << std::endl;
                        //
                        //						}else{
                        ////							std::cout << "Cell has no gaussian" <<
                        ///std::endl;
                        //						}
                        //					}
                        //					else{
                        ////						std::cout << "Cell NULL" << std::endl;
                        //					}

                        assert(score_tmp <= 1);
                        score += score_tmp;
                        //					std::cout << "Thus Score " << score << "\n" <<
                        //std::endl;
                    }
                    // 				std::cout << "\n";
                }
            }

            // 			std::cout << "Cell with gaussians " << gaussian_cells << std::endl;
            all_cell_gaussians = all_cell_gaussians + gaussian_cells;
            if (score == 0)
            {
                // Minimum score value if the score wasn't updated !
                score = 0.1;
            }

            //			std::cout << "FINAL likelihood " << score << " old " << score_old << " null vec
            //" << score_tmp_nul_vec << std::endl;
            particleCloud[i].SetLikelihood(score);
        }

        // 		std::cout << "All cell with gaussians " << all_cell_gaussians / particleCloud.size() <<
        // std::endl;
        if (all_cell_gaussians == 0)
        {
            exit(0);
        }

        //		for(unsigned int j = 0; j < ndts.size(); j++)
        //			delete ndts[j];

        Normalize();
        //		if(forceSIR) {
        //			std::cout << "FORCE SIR ALWAYS" << std::endl;
        ////			SIRUpdate();
        //			ModernSIRUpdate();
        //		}
        //		else{
        // see :
        // https://en.wikipedia.org/wiki/Particle_filter#Sequential_Importance_Resampling_(SIR)
        if (_use_new_SIR_update)
        {
            double sum_of_weights_squared = 0;
            double sum_weights = 0;
            assert(particleCloud.size() > 0);
            for (auto el : particleCloud)
            {
                sum_of_weights_squared = sum_of_weights_squared + (el.GetWeight() * el.GetWeight());
                sum_weights = sum_weights + el.GetWeight();
            }
            double n_effective = 1 / sum_of_weights_squared;

            std::cout << "There are " << n_effective << " particles effective out of "
                      << particleCloud.size() << " particles with a weight of sum "
                      << sum_of_weights_squared << " and a non squared sum of " << sum_weights
                      << " triggering under " << n_effective_under_which_trigger_SIR_update
                      << std::endl;

            assert(sum_of_weights_squared > 0);

            assert(n_effective > 0);
            assert(n_effective < particleCloud.size());

            if (n_effective < n_effective_under_which_trigger_SIR_update || sinceSIR > sirCount)
            {
                ModernSIRUpdate();
                sinceSIR = 0;
            }
            else
            {
                sinceSIR++;
            }
        }
        else
        {
            if (forceSIR)
            {
                std::cout << "FORCE SIR ALWAYS" << std::endl;
                //			SIRUpdate();
                ModernSIRUpdate();
            }
            else
            {
                double varP = 0;
                for (int i = 0; i < particleCloud.size(); i++)
                    varP +=
                        (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size())) *
                        (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
                varP /= double(particleCloud.size());
                varP = sqrt(varP);
                if (varP > varLimit || sinceSIR > sirCount)
                {
                    // fprintf(stderr,"-SIR- ");
                    sinceSIR = 0;
                    //				SIRUpdate();
                    ModernSIRUpdate();
                }
                else
                    sinceSIR++;
            }
        }

        // 		exit(0);
    }

    // 	std::cout << " Wahat " << ret_done << " == " << false << " && "<< no_mov << " == " << false
    // << std::endl; 	if(ret_done == false && no_mov == false){ 		std::cout << "WHAT NO PARTICULE
    // TESTED ?" << std::endl; 		exit(0);
    // 	}
    //
    // 	std::cout << "RET \n" << ret.matrix() << std::endl;

    return ret;
    // 	std::cout << "END" << std::endl;
}

std::tuple<double, double> perception_oru::particle_filter::hybridScoring(
    const perception_oru::NDTCell& ndts_cell, const Eigen::Vector3d& m, const Eigen::Affine3d& T)
{

    //	double scaling = 0.7 / 4; //0.7 is where (exp(-x) = 0.5 and 4 is the result for obstacle at
    //2 m difference
    double cx, cy, cz;
    ndtMap->getCellSizeInMeters(cx, cy, cz);
    double minval = std::min(cx, std::min(cy, cz));
    double scaling =
        4 / (_cell_neighbor_to_consider *
             minval); // Hence the pdf will be zero if the cell squared distance is outside the zone

    //	std::cout << "Distance at 0 " << _cell_neighbor_to_consider * minval << std::endl;

    pcl::PointXYZL p;
    p.x = m[0];
    p.y = m[1];
    p.z = m[2];
    double score = 0;
    double nb_of_gaussians = 0;

    perception_oru::NDTCell* cell_closest;
    ndtMap->getCellAtPoint(p, cell_closest);
    if (cell_closest != NULL && cell_closest->hasGaussian_)
    {

        nb_of_gaussians = 1;
        double distance = (m - cell_closest->getMean()).norm();
        //		score = 0.1 + 0.9 * exp(- scaling * distance * distance );
        score = 0.1 + 0.9 * exp(-scaling * distance);
        //
        ////		std::cout << "Single cell " << std::endl;
        //		nb_of_gaussians = 1;
        ////						cells.push_back(cell_closest);
        //		Eigen::Matrix3d map_cov = cell_closest->getCov();
        //		//cell->blurCov( 10 );
        //		//Eigen::Matrix3d covCombined = cell->getCov() + T.rotation() * ndts[n]->getCov() *
        //T.rotation().transpose();
        ////								Eigen::Matrix3d covCombined = map_cov + T.rotation() * ndts[n]->getCov() *
        ///T.rotation().transpose();
        //		Eigen::Matrix3d covCombined =
        //				map_cov + T.rotation().transpose() * ndts_cell.getCov() * T.rotation();
        //		Eigen::Matrix3d icov;
        //		bool exists;
        //		double det = 0;
        //		//I guess this step takes forever... if too much cells are here... :(
        //		covCombined.computeInverseAndDetWithCheck(icov, det, exists);
        //		if (exists) {
        //			double l = (cell_closest->getMean() - m).dot(icov * (cell_closest->getMean() -
        //m));
        //
        //			if (_use_euclidean_for_long_distances) {
        ////				std::cout << "Use euclid longue distance " << std::endl;
        //				//We use euclidean if probabilistic is too small !
        //				double distance_euclid = (m - cell_closest->getMean()).norm();
        //				if (l <= distance_euclid) { //THis is mathematically incorrect !
        ////					std::cout << " Not Euclid " << l << std::endl;
        //					score =
        //							0.1 + 0.9 * exp(-_scaling_factor_gaussian * l /
        //							                2.0);
        ////					std::cout << "Return score : " << score << std::endl;
        //				} else {
        //
        ////					std::cout << "Euclid " << distance_euclid << std::endl;
        //					score =
        //							0.1 + 0.9 * exp(-_scaling_factor_gaussian * distance_euclid /
        //							                2.0);
        ////					std::cout << "Return score : " << score << std::endl;
        //				}
        //			} else {
        ////				std::cout << "Not Euclid longue dist " << l << std::endl;
        //				score = 0.1 + 0.9 * exp(-_scaling_factor_gaussian * l /
        //				                             2.0); // l is distance between means. 0.005 is
        //scaling down that distance simulating "bigger" Gaussians.
        /// 										double score_textbook += exp( -l / 2);
        ////				std::cout << "Return score : " << score << std::endl;
        //			}
        //
        //		}
        //		else{
        ////			std::cout << "Doesn't exist" << std::endl;
        //		}
    }
    else
    {
        //		std::cout << "Multiple cells" << std::endl;
        nb_of_gaussians = 1;
        auto cells = ndtMap->getCellsForPoint(p, _cell_neighbor_to_consider, true);
        //		ndtMap->getCellAtPoint(p, cell_for_test);

        double score_tmp_tmp = 0;
        if (cells.size() > 0)
        {
            for (auto cell : cells)
            {
                double distance = (m - cell->getMean()).norm();
                //				score_tmp_tmp += 0.1 + 0.9 * exp(- scaling * distance * distance );
                score_tmp_tmp += 0.1 + 0.9 * exp(-scaling * distance);
                //				std::cout << "Return score in : " << score_tmp_tmp << " " << distance <<
                //std::endl;
            }

            score_tmp_tmp = score_tmp_tmp / cells.size();
        }
        score = score_tmp_tmp;
        //		std::cout << "Return score : " << score << std::endl;
    }
    //	std::cout << "Return score : " << score << std::endl;
    return std::make_tuple(score, nb_of_gaussians);
}

void perception_oru::particle_filter::UpdateAndPredictEffRe(Eigen::Affine3d tMotion,
                                                            perception_oru::NDTMap* ndtLocalMap_,
                                                            double subsample_level,
                                                            double z_cut,
                                                            double x_var,
                                                            double y_var,
                                                            double th_var,
                                                            double r_x_var,
                                                            double r_y_var,
                                                            double r_th_var,
                                                            int tres)
{
    if (subsample_level < 0 || subsample_level > 1)
        subsample_level = 1;
    double tre = weights[tres];
    Eigen::Vector3d tr = tMotion.translation();
    Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);
    if (tr[0] != 0.0 || tr[1] != 0.0 || rot[2] != 0)
    {
        int all_cells;
        int subsample_cells;
        int used_cells = 0;
        Predict2D(tr[0], tr[1], rot[2], fabs(tr[0]) * x_var, fabs(tr[1]) * y_var,
                  fabs(rot[2]) * th_var); // this line!!!!!!!!!!!!!!!!!!!!!!!!!
        std::vector<perception_oru::NDTCell*> ndts0 = ndtLocalMap_->getAllCells();
        std::vector<perception_oru::NDTCell*> ndts;
        if (subsample_level != 1)
        {
            srand(time(NULL));
            for (int i = 0; i < ndts0.size(); ++i)
            {
                double p = ((double)rand()) / RAND_MAX;

                if (p < subsample_level && ndts0[i]->getMean()[2] > z_cut &&
                    ndts0[i]->getClass() != perception_oru::NDTCell::HORIZONTAL)
                    ndts.push_back(ndts0[i]);
                else
                    delete ndts0[i];
            }
        }
        else
            ndts = ndts0;

        //#pragma omp parallel for
        for (int i = 0; i < particleCloud.size(); i++)
        {
            Eigen::Affine3d T;
            if (particleCloud[i].GetProbability() > tre)
            {
                T = particleCloud[i].GetAsAffine();
            }
            else
            {
                std::default_random_engine generator;
                std::normal_distribution<double> distribution_x(m_pose[0], r_x_var);
                std::normal_distribution<double> distribution_y(m_pose[1], r_y_var);
                std::normal_distribution<double> distribution_t(m_pose[2], r_th_var);
                particleCloud[i].Set(0.0, 0.0, distribution_t(generator), distribution_x(generator),
                                     distribution_y(generator), 0.0);

                T = particleCloud[i].GetAsAffine();
            }
            // std::vector<perception_oru::NDTCell*> ndts;
            // ndts = ndtLocalMap_->pseudoTransformNDT(T);
            double score = 1;
            if (ndts.size() == 0)
                fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
            for (int n = 0; n < ndts.size(); n++)
            {
                Eigen::Vector3d m = T * ndts[n]->getMean();
                // if(m[2] < zfilt_min) continue;
                perception_oru::NDTCell* cell;
                pcl::PointXYZL p;
                p.x = m[0];
                p.y = m[1];
                p.z = m[2];
                if (ndtMap->getCellAtPoint(p, cell))
                {
                    if (cell == NULL)
                        continue;
                    if (cell->hasGaussian_)
                    {
                        Eigen::Matrix3d covCombined = cell->getCov() + T.rotation() *
                                                                           ndts[n]->getCov() *
                                                                           T.rotation().transpose();
                        Eigen::Matrix3d icov;
                        bool exists;
                        double det = 0;
                        covCombined.computeInverseAndDetWithCheck(icov, det, exists);
                        if (!exists)
                            continue;
                        double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
                        if (l * 0 != 0)
                            continue;
                        score += 0.1 + 0.9 * exp(-0.5 * l / 2.0);
                    }
                    else
                    {
                    }
                }
            }
            particleCloud[i].SetLikelihood(score);
        }
        for (unsigned int j = 0; j < ndts.size(); j++)
            delete ndts[j];

        Normalize();
        if (forceSIR)
        {
            std::cout << "FORCE SIR ALWAYS" << std::endl;
            //			SIRUpdate();
            ModernSIRUpdate();
        }
        else
        {
            // see :
            // https://en.wikipedia.org/wiki/Particle_filter#Sequential_Importance_Resampling_(SIR)
            double sum_of_weights_squared = 0;
            for (auto el : particleCloud)
            {
                sum_of_weights_squared = sum_of_weights_squared + (el.GetWeight() * el.GetWeight());
            }
            double n_effective = 1 / sum_of_weights_squared;

            std::cout << "There are " << n_effective << " particles effective out of "
                      << particleCloud.size() << " particles" << std::endl;
            if (n_effective < n_effective_under_which_trigger_SIR_update || sinceSIR > sirCount)
            {
                ModernSIRUpdate();
                sinceSIR = 0;
            }
            else
            {
                sinceSIR++;
            }

            //			double varP = 0;
            //			for(int i = 0; i < particleCloud.size(); i++)
            //				varP += (particleCloud[i].GetProbability() - 1.0 /
            //double(particleCloud.size()))
            //				        * (particleCloud[i].GetProbability() - 1.0 /
            //double(particleCloud.size())); 			varP /= double(particleCloud.size()); 			varP =
            //sqrt(varP); 			if(varP > varLimit || sinceSIR > sirCount){
            //				//fprintf(stderr,"-SIR- ");
            //				sinceSIR = 0;
            ////				SIRUpdate();
            //				ModernSIRUpdate();
            //			}else
            //				sinceSIR++;
        }
    }
}

void perception_oru::particle_filter::Predict2D(
    double x, double y, double th, double sx, double sy, double sth)
{
    float dx = 0.0, dy = 0.0, dl = 0.0;
    float t = 0.0;
    float dxe, dye; ///< estimates
    std::default_random_engine generator;
    float eps = 0.000001;
    for (int i = 0; i < particleCloud.size(); i++)
    {
        double px, py, pz, pr, pp, pt;
        particleCloud[i].GetXYZ(px, py, pz);
        particleCloud[i].GetRPY(pr, pp, pt);

        std::normal_distribution<double> distribution_x(0, sx);
        std::normal_distribution<double> distribution_y(0, sy);
        std::normal_distribution<double> distribution_th(0, sth);
        // Generate noise from normal distribution
        dxe = x + distribution_x(generator);
        dye = y + distribution_y(generator);

        // if(fabs(x)<eps && fabs(y)<eps){
        //	dxe = x*0;
        //	dye = y*0;
        //
        // }

        dl = sqrt(dxe * dxe + dye * dye);

        // if(fabs(dxe)<eps && fabs(dye)<eps)
        // t = 0;
        // else
        t = atan2(dye, dxe);

        dx = dl * cos(pt + t);
        dy = dl * sin(pt + t);
        // ROS_INFO_STREAM("th"<<th);
        px += dx;
        py += dy;
        pt = pt + th + distribution_th(generator);
        toPI(pt);
        particleCloud[i].Set(pr, pp, pt, px, py, pz);
    }
    //    isAvgSet = false;
}
void perception_oru::particle_filter::to2PI(double& a)
{
    a = (double)fmod((double)(a), (double)(2 * M_PI));
    if (a < 0)
        a += 2 * (double)M_PI;
}
void perception_oru::particle_filter::toPI(double& a)
{
    if (a > M_PI)
        while (a > M_PI)
            a -= 2.0 * M_PI;
    else if (a < -M_PI)
        while (a < -M_PI)
            a += 2.0 * M_PI;
}

void perception_oru::particle_filter::Normalize(bool update_probabilities)
{

    std::cout << "NORMALIZE" << std::endl;

    double summ = 0;
    double sumX = 0, sumY = 0;
    double sumW = 0;
    double ax = 0, ay = 0;

    double sum_of_weights = 0;

    // isAvgSet = false;
    for (int i = 0; i < particleCloud.size(); i++)
    {
        if (update_probabilities)
        {
            //			if(particleCloud[i].GetProbability() * particleCloud[i].GetLikelihood() >
            //1){ 				std::cout << " PB with likelihood: " << particleCloud[i].GetProbability() << " * "
            //<< particleCloud[i].GetLikelihood() << std::endl;
            //			}
            particleCloud[i].SetProbability(particleCloud[i].GetProbability() *
                                            particleCloud[i].GetLikelihood());
        }
        summ += particleCloud[i].GetProbability();
    }
    std::cout << "GO" << std::endl;
    if (summ != 0)
    {
        for (int i = 0; i < particleCloud.size(); i++)
        {
            if (particleCloud[i].GetProbability() / summ > 1)
            {
                std::cout << " PB with weight: " << particleCloud[i].GetProbability() << " / "
                          << summ << std::endl;
            }
            particleCloud[i].SetProbability(particleCloud[i].GetProbability() / summ);

            // see :
            // https://en.wikipedia.org/wiki/Particle_filter#Sequential_Importance_Resampling_(SIR)
            if (particleCloud[i].GetWeight() * particleCloud[i].GetProbability() == 0)
            {
                std::cout << "Pb at i " << i << " " << particleCloud[i].GetWeight() << "*"
                          << particleCloud[i].GetProbability() << std::endl;
                assert(false);
            }
            //			if(update_probabilities) {
            particleCloud[i].SetWeight(particleCloud[i].GetWeight() *
                                       particleCloud[i].GetProbability());
            //			}
            assert(particleCloud[i].GetWeight() > 0);
            sum_of_weights += particleCloud[i].GetWeight();

            weights[i] = particleCloud[i].GetProbability();
            double x, y, z, r, p, t;
            particleCloud[i].GetXYZ(x, y, z);
            particleCloud[i].GetRPY(r, p, t);
            sumX += particleCloud[i].GetProbability() * x;
            sumY += particleCloud[i].GetProbability() * y;
            ax += particleCloud[i].GetProbability() * cos(t);
            ay += particleCloud[i].GetProbability() * sin(t);
            sumW += particleCloud[i].GetProbability();
        }
    }
    else
    {
        for (int i = 0; i < particleCloud.size(); i++)
        {
            if (1.0 / particleCloud.size() > 1)
            {
                std::cout << " PB with summ == 0 : " << 1.0 / particleCloud.size() << " / " << summ
                          << std::endl;
            }
            particleCloud[i].SetProbability(1.0 / particleCloud.size());

            if (particleCloud[i].GetWeight() * particleCloud[i].GetProbability() == 0)
            {
                std::cout << "Pb at i " << i << " " << particleCloud[i].GetWeight() << "*"
                          << particleCloud[i].GetProbability() << std::endl;
                assert(false);
            }
            //			if(update_probabilities) {
            particleCloud[i].SetWeight(particleCloud[i].GetWeight() *
                                       particleCloud[i].GetProbability());
            //			}
            assert(particleCloud[i].GetWeight() > 0);
            sum_of_weights += particleCloud[i].GetWeight();

            weights[i] = particleCloud[i].GetProbability();
            double x, y, z, r, p, t;
            particleCloud[i].GetXYZ(x, y, z);
            particleCloud[i].GetRPY(r, p, t);
            sumX += particleCloud[i].GetProbability() * x;
            sumY += particleCloud[i].GetProbability() * y;
            ax += particleCloud[i].GetProbability() * cos(t);
            ay += particleCloud[i].GetProbability() * sin(t);
            sumW += particleCloud[i].GetProbability();
        }
    }

    // normalizing the weights
    std::cout << "Sum of weights : " << sum_of_weights << std::endl;
    double tmp_weight = 0;
    for (int i = 0; i < particleCloud.size(); i++)
    {
        //		std::cout << "normalize weight " << particleCloud[i].GetWeight() << "/" <<
        //sum_of_weights << " = " << particleCloud[i].GetWeight() / sum_of_weights << std::endl;
        tmp_weight = tmp_weight + (particleCloud[i].GetWeight() / sum_of_weights);
        //		std::cout << "before " << particleCloud[i].GetWeight() << std::endl;
        particleCloud[i].SetWeight(particleCloud[i].GetWeight() / sum_of_weights);
        //		std::cout << "After " << particleCloud[i].GetWeight() << std::endl;
        assert(particleCloud[i].GetWeight() > 0);
    }

    double nw_sum_tmp = 0;
    for (auto el : particleCloud)
    {
        nw_sum_tmp = nw_sum_tmp + el.GetWeight();
    }
    std::cout << "New sum : " << nw_sum_tmp << " == " << tmp_weight << " and old " << sum_of_weights
              << std::endl;
    //	assert(nw_sum_tmp == tmp_weight);

    m_pose << sumX, sumY, atan2(ay, ax);
    std::sort(weights.begin(), weights.end(), myfunction);
    // ROS_INFO_STREAM(weights.front()<<" "<< weights.back());
}
void perception_oru::particle_filter::SIRUpdate()
{

    std::cout << "\n DOING THE SIR UPDATE ******************************** \n" << std::endl;

    std::vector<particle, Eigen::aligned_allocator<particle>> tmp2;
    std::vector<particle, Eigen::aligned_allocator<particle>> tmp = particleCloud;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist(0, 1);
    double U = 0, sum_proba_particles_seen = 0;
    int index_tmp = 0, index_particle_cloud = 0;
    //	int k = 0;
    U = dist(generator) / (double)particleCloud.size();
    std::cout << "SIRUpdate()::U= " << U << std::endl;
    while (U < 1.0)
    {
        if (sum_proba_particles_seen > U)
        {
            U += 1.0 / (double)particleCloud.size();
            //			if(k >= particleCloud.size() || index_tmp >= particleCloud.size()){
            if (index_particle_cloud >= particleCloud.size() || index_tmp >= particleCloud.size())
            {
                while (index_tmp < particleCloud.size())
                {
                    tmp[index_tmp] = particleCloud[particleCloud.size() - 1];
                    tmp[index_tmp].probability = 1.0 / (double)particleCloud.size();
                    index_tmp++;
                }
                // fprintf(stderr,"ERROR: SIRupdate:: Invalid index k='%d' or i='%d'\n",k,i);
                tmp2 = particleCloud;
                particleCloud = tmp;
                tmp = tmp2;
                return;
            }
            //			tmp[index_tmp] = particleCloud[k];
            tmp[index_tmp] = particleCloud[index_particle_cloud];
            tmp[index_tmp].probability = 1.0 / (double)particleCloud.size();
            index_tmp++;
        }
        else
        {
            index_particle_cloud++;
            //			k = index_particle_cloud; //<<- effectively making j and k the same variable. I
            //can probably remove j
            if (index_particle_cloud >= particleCloud.size())
            {
                // If we have seen the whole particle cloud we populate the tmp with the last
                // particle with a standard proba and return ? :/
                while (index_tmp < particleCloud.size())
                {
                    tmp[index_tmp] = particleCloud[particleCloud.size() - 1];
                    tmp[index_tmp].probability = 1.0 / (double)particleCloud.size();
                    index_tmp++;
                }
                // fprintf(stderr,"ERROR: SIRupdate:: Invalid index j='%d' \n",j);
                tmp2 = particleCloud;
                particleCloud = tmp;
                tmp = tmp2;
                return;
            }

            // Add to sum of proba
            sum_proba_particles_seen += particleCloud[index_particle_cloud].probability;
            // j++; ///WAS HERE until 30.7.2008 <- ok yes I can remove k :|

            // If we have seen the whole particle cloud (already above with a return so should not
            // get here). Effectively the same thing
            if (index_particle_cloud == particleCloud.size())
            {
                throw std::runtime_error("I belived this should be innacessible. I was wrong. In "
                                         "SIR UPDATE ndtLocallization");
                while (index_tmp < particleCloud.size())
                {
                    //					tmp[index_tmp] = particleCloud[k - 1];
                    tmp[index_tmp] = particleCloud[index_particle_cloud - 1];
                    tmp[index_tmp].probability = 1.0 / (double)particleCloud.size();
                    index_tmp++;
                }
                tmp2 = particleCloud;
                particleCloud = tmp;
                tmp = tmp2;
                return;
            }
        }
    } // While
    while (index_tmp < particleCloud.size())
    {
        //		if(k >= particleCloud.size()) k = particleCloud.size() - 1;
        //		tmp[index_tmp] = particleCloud[k];
        if (index_particle_cloud >= particleCloud.size())
            index_particle_cloud = particleCloud.size() - 1;
        tmp[index_tmp] = particleCloud[index_particle_cloud];
        tmp[index_tmp].probability = 1.0 / (double)particleCloud.size();
        index_tmp++;
    }
    //  isAvgSet = false;
    tmp2 = particleCloud;
    particleCloud = tmp;
    tmp = tmp2;
}

/**
 * As presented in
 * https://en.wikipedia.org/wiki/Particle_filter#Sequential_Importance_Resampling_(SIR)
 */
void perception_oru::particle_filter::ModernSIRUpdate()
{

    std::cout << "\n DOING THE MODERN SIR UPDATE ******************************** \n" << std::endl;

    std::vector<particle, Eigen::aligned_allocator<particle>> particleCloud_tmp = particleCloud;

    bool do_SIR = false;
    if (do_SIR == true)
    {
        std::cout << "Sort particles " << std::endl;
        // Sort by weight probability difference
        std::sort(particleCloud_tmp.begin(), particleCloud_tmp.end(),
                  [](particle const& a, particle const& b) {
                      return a.GetProbability() < b.GetProbability();
                  });
        for (int i = 0; i < particleCloud_tmp.size(); ++i)
        {
            if (i < particleCloud_tmp.size() - 1)
            {
                assert(particleCloud_tmp[i].GetProbability() <=
                       particleCloud_tmp[i + 1].GetProbability());
            }
            particleCloud_tmp[i].index_proba = i;
        }

        std::sort(
            particleCloud_tmp.begin(), particleCloud_tmp.end(),
            [](particle const& a, particle const& b) { return a.GetWeight() < b.GetWeight(); });
        for (int i = 0; i < particleCloud_tmp.size(); ++i)
        {
            if (i < particleCloud_tmp.size() - 1)
            {
                assert(particleCloud_tmp[i].GetWeight() <= particleCloud_tmp[i + 1].GetWeight());
            }
            particleCloud_tmp[i].index_weight = i;
        }
        // Sort: lower is particle with similar index for proba and weight. Higher are different.
        std::sort(particleCloud_tmp.begin(), particleCloud_tmp.end(),
                  [](particle const& a, particle const& b) {
                      return (std::abs(a.index_proba - a.index_weight) <
                              std::abs(b.index_proba - b.index_weight));
                  });
    }
    else
    {
        // Clasic MCL
        std::sort(particleCloud_tmp.begin(), particleCloud_tmp.end(),
                  [](particle const& a, particle const& b) {
                      return a.GetProbability() > b.GetProbability();
                  });
        for (int i = 0; i < particleCloud_tmp.size(); ++i)
        {
            if (i < particleCloud_tmp.size() - 1)
            {
                assert(particleCloud_tmp[i].GetProbability() >=
                       particleCloud_tmp[i + 1].GetProbability());
            }
        }
    }

    std::cout << "GetFirst half " << std::endl;
    // Get N particle for the set with proportional weights/probability
    std::vector<particle, Eigen::aligned_allocator<particle>> new_particle_set_tmp(
        particleCloud_tmp.begin(), particleCloud_tmp.begin() + particleCloud.size() / 2);
    std::vector<particle, Eigen::aligned_allocator<particle>> new_particle_set;

    std::default_random_engine generator;

    std::cout << "Duplicate and resampling step " << std::endl;
    std::cout << "Motion model " << var_for_new_particle << " " << _motion_model_cov_y << std::endl;
    // Duplicate the particles and keep going
    for (auto el : new_particle_set_tmp)
    {

        double new_weight = 1.0 / (double)new_particle_set_tmp.size();
        assert(new_weight > 0);
        el.SetWeight(new_weight);

        //		if(do_SIR == false) {
        // Reset all particles
        el.SetProbability(1 / (double)particleCloud.size());
        ///    std::cout << "ADDING PROBA OF " << probability << std::endl;
        el.SetLikelihood(1);
        el.SetWeight(1);
        //		}

        new_particle_set.push_back(el);

        // Add noise to this particle !
        double x, y, z;
        el.GetXYZ(x, y, z);
        double roll, pitch, yaw;
        el.GetRPY(roll, pitch, yaw);
        std::normal_distribution<double> distribution_x(x, var_for_new_particle);
        std::normal_distribution<double> distribution_y(y, var_for_new_particle);
        //		std::normal_distribution<double> distribution_t(th, th * var);
        double xx = distribution_x(generator);
        double yy = distribution_y(generator);
        particle el2 = el;
        el2.Set(roll, pitch, yaw, xx, yy, z);
        new_particle_set.push_back(el2);
    }

    assert(new_particle_set.size() == particleCloud.size());

    particleCloud = new_particle_set;

    Normalize(false);
}

void perception_oru::particle_filter::GetRandomPoint(perception_oru::NDTCell* cell,
                                                     double& x,
                                                     double& y,
                                                     double& th)
{
    Eigen::Vector3d mean = cell->getMean();
    Eigen::Matrix3d cov = cell->getCov();
    double mX = mean[0], mY = mean[1], mTh = mean[2];
    double varX = cov(0, 0), varY = cov(1, 1), varTh = cov(2, 2);
    std::default_random_engine generator;

    std::normal_distribution<double> distribution_x(mX, varX);
    std::normal_distribution<double> distribution_y(mY, varY);
    std::normal_distribution<double> distribution_th(mTh, varTh);
    x = distribution_x(generator);
    y = distribution_y(generator);
    th = distribution_th(generator);
}

void perception_oru::particle_filter::EigenSort(Eigen::Vector3d& eigenvalues,
                                                Eigen::Matrix3d& eigenvectors)
{
    int k, j, i;
    double p;

    for (i = 0; i < 2; i++)
    {
        p = eigenvalues(k = i);
        for (j = i + 1; j < 3; j++)
            if (fabs(eigenvalues(j)) >= fabs(p))
                p = eigenvalues(k = j);
        if (k != i)
        {
            eigenvalues.row(k).swap(eigenvalues.row(i));
            eigenvectors.col(k).swap(eigenvectors.col(i));
        }
    }
}

Eigen::Affine3d perception_oru::particle_filter::getAsAffine(float x, float y, float yaw)
{
    Eigen::Matrix3d m;

    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d v(x, y, 0);
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    T.translation() << x, y, 0;
    return T;
}
