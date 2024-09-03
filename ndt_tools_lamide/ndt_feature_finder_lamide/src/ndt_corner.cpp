#include "ndt_feature_finder_lamide/ndt_corner.hpp"



std::vector< perception_oru::ndt_feature_finder::NDTCornerBundle > perception_oru::ndt_feature_finder::NDTCorner::getAllCorners(const perception_oru::NDTMap& map, const Eigen::Vector3d& robot_pose)
{
	auto corners_temp = getAllCorners(map);
	std::vector< NDTCornerBundle > corners;




	return corners;
}

std::vector< perception_oru::ndt_feature_finder::NDTCornerBundle > perception_oru::ndt_feature_finder::NDTCorner::getAllCorners(const perception_oru::NDTMap& map)
{

	clear();
	//Go throught the map and test every cell
	map.getCellSizeInMeters(_x_cell_size, _y_cell_size, _z_cell_size);

// 	std::cout << "Getting all cells " << map.numberOfActiveCells() << std::endl;
	auto allCells = map.getAllCellsShared();

// 	std::cout << "vector" << std::endl;
	assert(allCells.size() == map.numberOfActiveCells());
// 	std::cout << "vector" << std::endl;
	std::vector< NDTCornerBundle > out;
// 	std::cout << "vector" << std::endl;
	for(size_t i = 0 ; i < allCells.size() ; ++i){
// 		std::cout << "Searching cell nb " << i << std::endl;
		NDTCornerBundle corner_out;
// 	std::cout << "vector" << std::endl;
		if(cellIsCorner(map, *allCells[i], allCells, corner_out) == true){
// 			std::cout << "It's a corner :)" << std::endl;
			out.push_back(corner_out);
// 			_corners_position.push_back( corner_out.getMean() );
// 			assert(_angles.size() == out.size());
		}
		else{
// 			std::cout << "Not a corner" << std::endl;
		}
	}

// 	assert(_angles.size() == _corners.size());
	_corners = out;

	clearAccurateCorner();
	//Translate the corner into openCV structure
// 	toOpenCV();

	return out;

}


bool perception_oru::ndt_feature_finder::NDTCorner::cellIsCorner(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell, const std::vector< boost::shared_ptr< perception_oru::NDTCell > >& allCells, NDTCornerBundle& corner)
{
// 	std::cout << "is it a corner?" << std::endl;
	double angle;
	int count_of_good_wall = 0;

	try{
		//Calculate vector
		//Get all cells with a gaussian
		//For every cell get all other cells around and test if at least two are pointing toward the center cell
		//Get neighbor of the cell
// 		std::cout << "Getting closest cells" <<std::endl;

		std::vector< boost::shared_ptr< perception_oru::NDTCell > > neighbor;
		std::vector< boost::shared_ptr< perception_oru::NDTCell > > cells_initialized;

		getClosestCells(map, cell, _neighbor_size, neighbor, cells_initialized);

// 		std::cout << "Got the closest cells : " << neighbor.size() <<std::endl;
		assert(neighbor.size() <= allCells.size());

		if(neighbor.size() >= 8){
// 			std::cout << "CEnter " << cell.getMean() << std::endl;
			for(size_t j = 0 ; j < neighbor.size() ; ++j){
// 				std::cout << "Cells " << neighbor[j]->getMean() << std::endl;
// 				std::cout << "CEnter " << cell.getMean() << std::endl;

				for(size_t jj = j + 1 ; jj < neighbor.size() ; ++jj){

// 					std::cout << "Cells " << neighbor[jj]->getMean() << std::endl;
					if (neighbor[j]->getMean() == neighbor[jj]->getMean() && j != jj){
// 						std::cout << "Already SEEN " <<j << " and " << jj << std::endl;
						assert(neighbor[j]->getMean() != neighbor[jj]->getMean());
					}
				}
			}
		}

// 		std::cout << "Test same cell twice: " << std::endl;
// 		std::cout << std::endl << "ALL CELLS" << std::endl << std::endl;
		if(allCells.size() >= 8){
// 			std::cout << "CEnter " << cell.getMean() << std::endl;
			for(size_t j = 0 ; j < allCells.size() ; ++j){
// 				std::cout << "Cells " << allCells[j]->getMean() << std::endl;
				for(size_t jj = j + 1 ; jj < allCells.size() ; ++jj){
					if (allCells[j]->getMean() == allCells[jj]->getMean() && j != jj){
// 						std::cout << "Already SEEN " <<j << " and " << jj << std::endl;
						assert(allCells[j]->getMean() != allCells[jj]->getMean());
					}
				}
			}
		}

// 		exit(0);


// 		assert(neighbor.size() <= 8);

		//If there is more than one neighbor == Crossing
		if(neighbor.size() > 1){
// 			auto possible_neighbor = getCellsPointingToward(neighbor, cell);
			auto possible_neighbor = neighbor;
			if(possible_neighbor.size() > 0){
// 				std::cout << "nieghbor size " << possible_neighbor.size() << std::endl;
				if(gotAngledNDT(map, possible_neighbor, cells_initialized, corner)){
					return true;
				}
			}
			else{
// 				std::cout << "Not neighbor facing the right direction" << std::endl;
			}
		}
		else{
// 			std::cout << "Neighbor is too small : " << neighbor.size() << std::endl;
		}
	}
	catch(const std::exception& ex){
		// speciffic handling for all exceptions extending std::exception, except
		// std::runtime_error which is handled explicitly
		std::cerr << "Error occurred: " << ex.what() << std::endl;
	}


	return false;
}





std::vector< perception_oru::NDTCell* > perception_oru::ndt_feature_finder::NDTCorner::getCellsPointingToward(std::vector< perception_oru::NDTCell* >& neighbor, const perception_oru::NDTCell& cell) const
{
//Check if the angle between the center and the neighbor is going toward the center == Angle of the vector mean|mean to reference is the same as neighbor variance to reference

	std::vector< perception_oru::NDTCell* > possible_neighbor;
	auto mean_cell = cell.getMean();

	for(size_t j = 0 ; j < neighbor.size() ; ++j){

		//Get orientation of neighbor
		Eigen::Vector3d eigenval;
		Eigen::Matrix3d eigenvec;
		int index = getBiggestEigenVector2D(*neighbor[j], eigenval, eigenvec);
		Eigen::Vector3d orientation_neighbor = eigenvec.col(index);

		//Get Vector from neighbor to center
		auto mean_tmp = neighbor[j]->getMean();
		//Difference between both point with mean_tmp as start
		Eigen::Vector3d vec = mean_cell - mean_tmp;

		// angle
		double angle_tmp = getAngle(vec, orientation_neighbor);

		//Move angle into [0 ; PI]
		if(angle_tmp > M_PI){
			angle_tmp = (M_PI * 2) - angle_tmp;
		}

		// Corner if it is close to 0deg or 180deg
		if(angle_tmp < 0.1 || angle_tmp > M_PI - 0.1){
			possible_neighbor.push_back(neighbor[j]);
		}
	}

	return possible_neighbor;
}


bool perception_oru::ndt_feature_finder::NDTCorner::gotAngledNDT(const perception_oru::NDTMap& map, const std::vector< boost::shared_ptr< perception_oru::NDTCell > >& neighbor, const std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_initialized, NDTCornerBundle& corner) {

	for(size_t j = 0 ; j < neighbor.size() ; ++j){
		//Get orientation of neighbor
// 		std::cout << " j " << j <<std::endl;


		for(size_t i = j + 1 ; i < neighbor.size() ; ++i){

			assert(neighbor[j] != neighbor[i]);

				// FOR TESTING ONLY - TO REMOVE

				Eigen::Vector3d eigenval;
				Eigen::Matrix3d eigenvec;
				int index = getBiggestEigenVector2D(*neighbor[j], eigenval, eigenvec);
				Eigen::Vector3d orientation = eigenvec.col(index);
				auto mean_cell = neighbor[j]->getMean();
				Eigen::Vector3d eigenval_tmp;
				Eigen::Matrix3d eigenvec_tmp;
				int index_tmp = getBiggestEigenVector2D(*neighbor[i], eigenval_tmp, eigenvec_tmp);
				Eigen::Vector3d orientation_tmp = eigenvec_tmp.col(index_tmp);
				Eigen::Vector3d collision_point_old = collisionRay(orientation_tmp, neighbor[i]->getMean(), orientation, neighbor[j]->getMean());
//
				//END OF TEST

			Eigen::Vector3d collision_point = collisionNDTCells(*neighbor[j], *neighbor[i]);
//
				// FOR TESTING ONLY - TO REMOVE
				assert(collision_point_old == collision_point);

//
				auto mean_cell_tmp = neighbor[i]->getMean();
				auto vector1 = mean_cell - collision_point;
				auto vector2 = mean_cell_tmp - collision_point;
				double angle_from = atan2(vector1(1), vector1(0)) - atan2(0, 1);
				if (angle_from < 0) angle_from += 2 * M_PI;
				double angle_to = atan2(vector2(1), vector2(0)) - atan2(0, 1);
				if (angle_to < 0) angle_to += 2 * M_PI;
				double direction_test = (angle_to + angle_from) / 2;
//
				double angle_between_test = atan2(vector1(1), vector1(0)) - atan2(vector2(1), vector2(0));
				if (angle_between_test < 0) angle_between_test += 2 * M_PI;
//
//
// // 				std::cout << vector1 << "\n\n "<< orientation << "\n\n " << vector2 << " \n\n"<< orientation_tmp << std::endl;
// // 				std::cout << "means " << neighbor[i]->getMean() << " " << mean_cell_tmp << " and " <<neighbor[j]->getMean() << " " << mean_cell << " corner " << corner << std::endl << std::endl;;
// // 				int a;
// // 				std::cin >> a;
//
// // 				double width = std::abs(angle_to - angle_from);
//
				if(angle_between_test > M_PI){
// 					std::cout << (2*M_PI) - width << " == " << angle_tmp << std::endl;
// 					assert((2*M_PI) - width <= angle_tmp + 0.1);
// 					assert((2*M_PI) - width >= angle_tmp - 0.1);
					angle_between_test = (M_PI * 2) - angle_between_test;
					direction_test - M_PI;
					if(direction_test < 0){
						direction_test = direction_test + (2 * M_PI);
					}
				}
//
//
				double angle_tmp_test = angle_between_test;
// // 				std::cout << angle_tmp << " == " << angle_tmp_test << std::endl;
// // 				assert(angle_tmp == angle_tmp_test);
//
// 				//Move angle into [0 ; PI]
				if(angle_tmp_test > M_PI){
					angle_tmp_test = (M_PI * 2) - angle_tmp_test;
				}
// 				std::cout << "nsize " << neighbor.size() << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	// 			std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
				// Corner if it is close to 90deg

			//END OF TEST

			double angle_tmp;
			double orientation_t;
			//Faster function, only calculate angle for testing
			angleNDTCells(*neighbor[j], *neighbor[i], collision_point, angle_tmp, orientation_t);
// 			orientations.push_back(orientation_t);


			//TEST
// 				assert(angle_tmp_test == angle_tmp);
// 				assert(direction_test == direction);
			//END OF TEST

			if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ||
				(angle_tmp < ( ( 3 * M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( 3 * M_PI / 2 ) - 0.1 ) )
			){

				std::vector<double> orientations;
				std::vector<double> angles;
				//Calculate orientations. Slower function so we only use it if we found an angle
// 				angleNDTCells(map, *neighbor[j], *neighbor[i], collision_point, angle_tmp, orientations);
				orientationNDTCells(map, *neighbor[j], *neighbor[i], collision_point, angle_tmp, orientation_t, angles, orientations);
// 				assert(orientations.size() >= 1);
				//Need to correct angle orientation depending on where does the robot see the angle from. Check on which direction with initialized cells with no gaussians. Add a direction there.


				//Saving information about the corner: the mean and the two gaussian that defined it.
				corner.setMean(collision_point);
				corner.push_back_cell1(neighbor[i]);
				corner.push_back_cell2(neighbor[j]);
				corner.setAngles(angles);
				corner.setOrientations(orientations);
				corner.gaussian();


// 						std::cout << "COLLSION
				return true;
// 					}

			}
		}
	}
	return false;
}


void perception_oru::ndt_feature_finder::NDTCorner::clearAccurateCorner()
{
	std::vector < NDTCornerBundle > tmp;
	std::vector<std::pair<double,double> > angles_tmp;
	for(size_t i = 0 ; i < _corners.size() ; ++i){
		bool seen = false;

		for(size_t j = 0 ; j < tmp.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			Eigen::Vector3d vec = tmp[j].getMean() - _corners[i].getMean();
			if( vec.norm() < _x_cell_size){
				seen = true;
			}
		}
		if(seen == false){
			tmp.push_back(_corners[i]);
// 			angles_tmp.push_back(_angles[i]);
		}
	}

	_corners.clear();
// 	_angles.clear();
	_corners = tmp;
// 	_angles = angles_tmp;
}

// void perception_oru::ndt_feature_finder::NDTCorner::toOpenCV()
// {
// 	_opencv_corners.clear();
// 	_opencv_corners_position.clear();
//
// 	//Get openCV point _corners
// 	auto it = _corners.begin();
// 	for(it; it != _corners.end() ; ++it){
// 		cv::Point2d p;
// 		p.x = (*it).getMean()(0);
// 		p.y = (*it).getMean()(1);
// 		_opencv_corners.push_back(p);
// 	}
// 	auto it_pos = _corners_position.begin();
// 	for(it_pos; it_pos != _corners_position.end() ; ++it_pos){
// 		cv::Point2d p;
// 		p.x = (*it_pos)(0);
// 		p.y = (*it_pos)(1);
// 		_opencv_corners_position.push_back(p);
// 	}
// // 	std::cout << "All done" << std::endl;
// }


void perception_oru::ndt_feature_finder::NDTCorner::calculateAngles(const perception_oru::NDTMap& map)
{

}

/**
 * return true if at least one ndt in the neighbor is aligned with the given cell
 */
bool perception_oru::ndt_feature_finder::NDTCorner::AlignedNDT(const std::vector <perception_oru::NDTCell* >& neighbor, const perception_oru::NDTCell& cell)
{
	Eigen::Vector3d eigenval;
	Eigen::Matrix3d eigenvec;
	int index = getBiggestEigenVector2D(cell, eigenval, eigenvec);
	Eigen::Vector3d orientation = eigenvec.col(index);

	for(size_t j = 0 ; j < neighbor.size() ; ++j){
		if(neighbor[j] != &cell){
			Eigen::Vector3d eigenval2;
			Eigen::Matrix3d eigenvec2;
			int index2 = getBiggestEigenVector2D(*neighbor[j], eigenval2, eigenvec2);
			Eigen::Vector3d orientation2 = eigenvec2.col(index2);
			double angle_between = atan2(orientation(1), orientation(0)) - atan2(orientation2(1), orientation2(0));
			if (angle_between < 0) angle_between += 2 * M_PI;

			//Around 0
			if(angle_between <= 0.349 || angle_between >= 5.934){
				//Around 180
				if(angle_between >= 2.793 && angle_between <=3.491){
					return true;
				}
			}

		}
	}
	return false;

}

