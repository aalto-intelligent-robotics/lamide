cmake_minimum_required(VERSION 2.8.3)
project(ndt_generic_lamide)

find_package(catkin REQUIRED COMPONENTS velodyne_pointcloud_oru cmake_modules angles tf_conversions velodyne_msgs laser_geometry sensor_msgs pcl_conversions pcl_ros)
find_package(Boost REQUIRED COMPONENTS serialization filesystem)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS velodyne_pointcloud_oru  velodyne_msgs  tf_conversions angles  laser_geometry sensor_msgs pcl_conversions pcl_ros
  LIBRARIES ndt_generic_lamide
  DEPENDS PCL
)

set(CMAKE_CXX_STANDARD 11)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

MESSAGE("Velodyne: ${velodyne_pointcloud_INCLUDE_DIRS}")

#add_library(${PROJECT_NAME}_LIB src/pointcloudXYZIR.cc)
#target_link_libraries(${PROJECT_NAME}_LIB
#                      ${catkin_LIBRARIES}
#                      ${YAML_CPP_LIBRARIES})


set (${PROJECT_NAME}_LIB_SRCS
  src/sensors_utils.cpp
  src/eigen_utils.cpp
  src/pcl_utils.cpp
  src/utils.cpp
  src/io.cpp
  src/sensors_utils.cpp
  src/motion_model_3d.cpp
  src/math_utils.cpp
  src/labels.cpp
  src/ndt_utils.cpp
  #src/pointcloudXYZIR.cc
)




add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${LIBXML2_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${PCL_LIBRARIES}
  #${PROJECT_NAME}_LIB
)

add_executable(eigen_test test/eigen_test.cpp)
target_link_libraries(eigen_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

