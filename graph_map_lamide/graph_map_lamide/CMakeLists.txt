cmake_minimum_required(VERSION 2.8.3)
project(graph_map_lamide)
#set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)


find_package(octomap REQUIRED HINTS <path to code>/src/lamide/octomap)
# MESSAGE(WARNING "OctoMap found " ${octomap_FOUND})
# MESSAGE(WARNING "include " ${OCTOMAP_INCLUDE_DIRS})
# MESSAGE(WARNING "lib " ${OCTOMAP_LIBRARIES})
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    genmsg
    tf
    tf_conversions
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    velodyne_pointcloud_oru
    velodyne_msgs
    geometry_msgs
    laser_geometry
    cmake_modules
    ndt_registration_lamide
    ndt_map_lamide
    ndt_localization_lamide
    ndt_rviz_lamide
    ndt_generic_lamide
    ndt_offline_lamide
    graph_map_custom_msgs_lamide
     #all4eham
    )
find_package(cmake_modules REQUIRED)
#find_package(Eigen REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)


find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()



catkin_package(
    CATKIN_DEPENDS
    roscpp
    tf
    tf_conversions
    cmake_modules
    laser_geometry
    pcl_ros
    eigen_conversions
    sensor_msgs
    geometry_msgs
    message_runtime
    velodyne_msgs
    velodyne_pointcloud_oru
    ndt_registration_lamide
    ndt_map_lamide
    ndt_rviz_lamide
    ndt_localization_lamide
    ndt_offline_lamide
    ndt_generic_lamide
    graph_map_custom_msgs_lamide
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/graph_map_lamide/graph_map_lamide.cpp
    src/graph_map_lamide/graph_map_lamide_navigator.cpp
    src/graph_map_lamide/map_node.cpp
    src/graph_map_lamide/map_type.cpp
    src/graph_map_lamide_fuser.cpp
    src/graph_map_lamide/reg_type.cpp
    src/ndt/ndt_map_type.cpp
    src/ndt/ndtd2d_reg_type.cpp
    src/ndt/ndtd2d_reg_type_lamide.cpp
    src/ndt/ndt_map_param.cpp
    src/graphfactory.cpp
    src/visualization/graph_plot.cpp
    src/visualization/graph_visualization.cpp
    src/lidarUtils/lidar_utilities.cpp
    src/graph_map_lamide/factor.cpp
    src/graph_map_lamide/graph_optimization.cpp
    src/ndt_dl/ndtdl_map_type.cpp
    src/ndt_dl/ndtdl_reg_type.cpp
    src/ndt_dl/ndtdl_map_param.cpp
    src/ndt_dl/point_curv3.cpp
    src/visualization/marker_vis.cpp
    src/graph_map_lamide/register.cpp
    src/utils.cpp
    #src/ndt_dl/ndt_matcher_d2d_n.cpp
    )


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
#!add_dependencies(${PROJECT_NAME} graph_map_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}  ${Boost_LIBRARIES} pcl_common pcl_octree)

add_executable(velodyne_converter_lamide src/VelodyneMotionCompensation.cpp )
target_link_libraries(velodyne_converter_lamide ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )

add_executable(show_map_lamide src/show_map.cpp )
target_link_libraries(show_map_lamide ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )

add_executable(graph_map_lamide_fuser_node src/graph_map_lamide_fuser_node.cpp )
target_link_libraries(graph_map_lamide_fuser_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  )

add_executable(graph_map_lamide_publisher src/graph_map_lamide_publisher.cpp )
target_link_libraries(graph_map_lamide_publisher ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  )

add_executable(graph_mapping_lamide_offline src/graph_mapping_lamide_offline.cpp)
target_link_libraries(graph_mapping_lamide_offline ${LIBXML2_LIBRARIES}  ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${PROJECT_NAME})


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

install(TARGETS graph_map_lamide_fuser_node
    velodyne_converter_lamide
    graph_mapping_lamide_offline
    show_map_lamide
    graph_map_lamide_publisher
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY maps/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        PATTERN "private" EXCLUDE
)
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        PATTERN "private" EXCLUDE
)
