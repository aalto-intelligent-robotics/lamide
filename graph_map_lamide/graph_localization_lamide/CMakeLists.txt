cmake_minimum_required(VERSION 2.8.3)
project(graph_localization_lamide)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    add_definitions(-std=c++11)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    velodyne_pointcloud_oru
    geometry_msgs
    cmake_modules
    ndt_registration_lamide
    ndt_offline_lamide
    ndt_map_lamide
    ndt_localization_lamide
    ndt_map_lamide
    ndt_generic_lamide
    graph_map_lamide
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf_conversions
    tf
    genmsg
    )
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS signals)
find_package(octomap REQUIRED HINTS <path to code>/src/lamide/octomap)
catkin_package(
    CATKIN_DEPENDS
    velodyne_pointcloud_oru
    graph_map_lamide
    ndt_registration_lamide
    ndt_map_lamide
    ndt_localization_lamide
    ndt_offline_lamide
    ndt_generic_lamide
    roscpp
    laser_geometry
    sensor_msgs
    tf_conversions
    tf
    geometry_msgs
    cmake_modules
    eigen_conversions
    pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/localization_type.cpp
    src/localization_factory.cpp
    src/mcl_ndt/mcl_ndt.cpp
    src/mcl_ndt/mcl_ndt_lamide.cpp
    src/mcl_ndt/mcl_ndtdl.cpp
    src/mcl_ndt/submap_mcl.cpp
    src/ukf_ndt/3d_ndt_ukf.cpp
    src/ukf_ndt/ukf_ndt.cpp
    src/ukf_ndt/ukf_reg.cpp
    src/reg_localization_type/reg_localization_type.cpp
    src/LocalisationHeatMap.cpp
    src/pose_queue.cpp
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  )

add_executable(ukf_test test/ukf_test.cpp )
target_link_libraries(ukf_test ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(ndt_mcl_localization_graph_lamide src/ndt_mcl_localization.cpp)
target_link_libraries(ndt_mcl_localization_graph_lamide ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(realtime_dl_localization_lamide src/realtime_localization.cpp)
target_link_libraries(realtime_dl_localization_lamide ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(graph_localization_lamide_offline src/graph_localization_lamide_offline.cpp)
target_link_libraries(graph_localization_lamide_offline ${PROJECT_NAME} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${JPEG_LIBRARIES})


install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)

install(TARGETS ndt_mcl_localization_graph_lamide graph_localization_lamide_offline ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
