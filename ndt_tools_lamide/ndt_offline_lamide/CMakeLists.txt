cmake_minimum_required(VERSION 2.8.3)
project(ndt_offline_lamide)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_BUILD_TYPE release )
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  ndt_fuser_lamide
  ndt_map_lamide
  ndt_registration_lamide
  ndt_visualisation_lamide
  ndt_rviz_lamide
  ndt_localization_lamide
  ndt_generic_lamide
  pcl_ros
  velodyne_msgs
  velodyne_pointcloud
  tf_conversions
  eigen_conversions
  tf
  genmsg
#  OpenCV
#  REQUIRED
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ndt_fuser_lamide ndt_localization_lamide ndt_map_lamide ndt_registration_lamide ndt_visualisation_lamide pcl_ros velodyne_msgs velodyne_pointcloud tf tf_conversions genmsg ndt_rviz_lamide
  DEPENDS
  LIBRARIES ${PROJECT_NAME}

)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(LibXml2 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS signals)
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DLINUX_OS)
#find_package(ndt_visualisation_lamide)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)
find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

FIND_PACKAGE(VTK REQUIRED NO_MODULE)
IF (VTK_FOUND)
  INCLUDE (${VTK_USE_FILE})
ELSE(VTK_FOUND)
  MESSAGE(FATAL_ERROR "Cannot build the executable without VTK. Please set the VTK_DIR")
ENDIF (VTK_FOUND)

include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} #${catkin_INCLUDE_DIRS} (inkludera andra ros-paket )
)

set (${PROJECT_NAME}_LIB_SRCS
 src/imu_prediction.cpp
 src/readpointcloud.cpp
 #src/gps_test.cpp
)



add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES})

add_executable(fuser3d_offline src/fuser3d_offline.cpp)
target_link_libraries(fuser3d_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} )

add_executable(test_reader test/test_reader.cpp)
target_link_libraries(test_reader ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${PROJECT_NAME})


add_executable(bag_converter src/bag_converter.cpp)
target_link_libraries(bag_converter ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${OpenCV_LIBRARIES})

add_executable(tf_writer src/tf_writer.cpp)
target_link_libraries(tf_writer ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${OpenCV_LIBRARIES})

add_executable(tf_writer_from_txt src/tf_writer_from_txt.cpp)
target_link_libraries(tf_writer_from_txt ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${OpenCV_LIBRARIES})


add_executable(bag_file_view src/bag_file_view.cpp)
target_link_libraries(bag_file_view ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

#add_executable(mcl_offline src/mcl_offline.cpp)
#target_link_libraries(mcl_offline ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

#add_executable(gps_test src/gps_test.cpp)
#target_link_libraries(gps_test ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${PROJECT_NAME_INCLUDE_DIRS})


add_executable(view_ndtmap src/view_ndtmap.cpp)
target_link_libraries(view_ndtmap ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ndt_offline_laser_lamide src/ndt_offline_laser_lamide.cpp)
target_link_libraries(ndt_offline_laser_lamide ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ndt_offline_laser_noodom_lamide src/ndt_offline_laser_noodom_lamide.cpp)
target_link_libraries(ndt_offline_laser_noodom_lamide ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ndt_offline_pointcloud_lamide src/ndt_offline_pointcloud_lamide.cpp)
target_link_libraries(ndt_offline_pointcloud_lamide ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(ndt_offline_gt_lamide src/ndt_offline_gt_lamide.cpp)
target_link_libraries(ndt_offline_gt_lamide ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

#add_executable(narf src/narf_test.cpp)
#target_link_libraries(narf ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)
