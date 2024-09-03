#include <ndt_map_lamide/ndt_map.h>

int main(int argc, char **argv){

	std::cout << "HI4" << std::endl;
	perception_oru::NDTMap* map = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
	map->initialize(0.0,0.0,0.0,10,10,10,0.6,0.4,0.2,0.7,0.3,0.8,1.0,0.5);
	std::cout << "bye4" << std::endl;
	delete map;

	std::string file = "/home/malcolm/ros_catkin_ws/lunar_ws/src/auto_complete_graph/jff_maps/basement2d_laser_gustav_map_r05.jff";
// 	map.loadFromJFF(file.c_str());
	double resolution = 0.5;
	auto mapGrid = new perception_oru::LazyGrid(resolution);
	perception_oru::NDTMap map_load(mapGrid);
	if(map_load.loadFromJFF(file.c_str()) < 0)
		std::cout << "File didn't load" << std::endl;
	std::cout << "File loaded" << std::endl;


	perception_oru::NDTMap* map2 = new perception_oru::NDTMap(map_load);



	std::cout << "Initialized cells " << map2->getAllInitializedCellsShared().size() << " == " << map_load.getAllInitializedCellsShared().size() << std::endl;
	assert(map2->getAllInitializedCellsShared().size() == map_load.getAllInitializedCellsShared().size());

	std::cout << "cells " << map2->getAllCells().size() << " == " << map_load.getAllCells().size() << std::endl;
	assert(map2->getAllCells().size() == map_load.getAllCells().size());

	double a, b, c;
	map2->getCellSizeInMeters(a, b, c);
	double a1, b1, c1;
	map_load.getCellSizeInMeters(a1, b1, c1);
	std::cout << a << " == " << a1 << std::endl;
	std::cout << b << " == " << b1 << std::endl;
	std::cout << c << " == " << c1 << std::endl;
	assert(a == a1);
	assert(b == b1);
	assert(c == c1);

	map2->getGridSizeInMeters(a, b, c);
	map_load.getGridSizeInMeters(a1, b1, c1);
	std::cout << a << " == " << a1 << std::endl;
	std::cout << b << " == " << b1 << std::endl;
	std::cout << c << " == " << c1 << std::endl;
	assert(a == a1);
	assert(b == b1);
	assert(c == c1);

	std::cout << "Copy done " << std::endl;

//	delete map_load;
	delete map2;

}



