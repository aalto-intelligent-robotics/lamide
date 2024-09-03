cmake_minimum_required(VERSION 2.8.3)
project(ndt_registration_lamide)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)
find_package(catkin REQUIRED COMPONENTS ndt_map_lamide ndt_generic_lamide tf tf_conversions pcl_ros)
find_package(Boost REQUIRED COMPONENTS program_options system)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS ndt_map_lamide ndt_generic_lamide tf tf_conversions pcl_ros velodyne_pointcloud_oru
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set (${PROJECT_NAME}_LIB_SRCS
	src/ndt_matcher_d2d.cpp
    src/ndt_matcher_d2d_lamide.cpp
	src/ndt_matcher_d2d_2d.cpp
 	src/ndt_matcher_d2d_feature.cpp
	src/ndt_matcher_p2d.cpp
	src/ndt_matcher_d2d_sc.cpp
    src/ndt_matcher_d2d_sc_2d.cpp
    src/ndt_matcher_d2d_n.cpp
    src/ndt_matcher_d2d_n_sc.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(simple_test_ndt_reg test/simple.cc)
target_link_libraries(simple_test_ndt_reg ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(d2d_deathmatch test/deathmatch.cc)
target_link_libraries(d2d_deathmatch ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(p2d_deathmatch test/p2d_deathmatch.cc)
target_link_libraries(p2d_deathmatch ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(check_reg test/check_reg.cc)
target_link_libraries(check_reg
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS simple_test_ndt_reg ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
