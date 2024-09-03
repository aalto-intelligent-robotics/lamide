# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
project(ndt_fuser_lamide)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rosconsole
#  cmake_modules
  ndt_registration_lamide
  ndt_map_lamide
  ndt_generic_lamide
  laser_geometry
  sensor_msgs
  visualization_msgs
  ndt_visualisation_lamide
  std_srvs
#  message_generation
)
#message(DEPRECATION "ndt_fuser_lamide is legacy code and is no longer developed. To build nd-maps, please use graph-map package.")

#find_package(OpenGL REQUIRED)
#include_directories(${OPENGL_INCLUDE_DIR})
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIR})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#add_definitions(-DNO_NDT_VIZ)
catkin_package(
    DEPENDS  Boost #OPENGL GLUT libxmu-dev libxi-dev
    CATKIN_DEPENDS ndt_registration_lamide ndt_map_lamide laser_geometry sensor_msgs ndt_visualisation_lamide std_srvs message_runtime visualization_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES}
)

#find_package(libmux REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED )
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${libmux_INCLUDE_DIRS})
#find_package(LibXml2 REQUIRED)
#include_directories(${LIBXML2_INCLUDE_DIR})
#ADD_DEFINITIONS(-DLINUX_OS)
#find_package(ndt_visualisation_lamide)
#find_package(GLU REQUIRED)

#find_package(JPEG REQUIRED)
#include_directories(${JPEG_INCLUDE_DIR})

#FIND_PACKAGE(MRPT REQUIRED gui;base;opengl)
#find_package(MRPT REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

#uncomment if you have defined messages
#add_message_files(
#  FILES
#  ndt_cell_lamide.msg
#  ndt_map_lamide.msg
#)
#uncomment if you have defined services
#add_service_files(
#  FILES

#)

#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#message(STATUS "->" ${MRPT_LIBS})
#message(STATUS "->" ${MRPT_LIBRARIES})

set (${PROJECT_NAME}_LIB_SRCS
	    src/ndt_fuser_lamide.cpp
	    src/ndt_fuser_hmt_lamide.cpp
	)

set (${PROJECT_NAME}_LIB_ROS_SRCS
	    src/ndt_fuser_ros_wrappers_src_lamide/ndt_fuser_init_lamide.cpp
	)

#add_executable(test_ndt_fuser_lamide src/ndt_fuser_from_depth_lamide.cpp)
#target_link_libraries(test_ndt_fuser_lamide ${LIBXML2_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES} ${OPENGL_LIBRARIES} ${MRPT_LIBS})
#target_link_libraries(test_ndt_fuser_lamide ${Boost_LIBRARIES})
#rosbuild_add_openmp_flags(test_ndt_fuser_lamide)

#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS} ${${PROJECT_NAME}_LIB_ROS_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ndt_fuser_node_lamide src/ndt_fuser_node_lamide.cpp)
# add_dependencies(ndt_fuser_node_lamide ndt_fuser_generate_messages_cpp_lamide)

target_link_libraries(ndt_fuser_node_lamide ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})


add_executable(ndt_fuser_logger_node_lamide src/ndt_fuser_logger_node_lamide.cpp)
#add_dependencies(ndt_fuser_logger_node_lamide ndt_fuser__generate_messages_cpp_lamide)

target_link_libraries(ndt_fuser_logger_node_lamide ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})


add_executable(init_test src/init_test.cpp)
#add_dependencies(init_test ndt_fuser__generate_messages_cpp_lamide)

target_link_libraries(init_test ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
install(TARGETS ndt_fuser_node_lamide ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
install(FILES
   launch/fuser_2dlaser.launch
   launch/fuser_hmt_vellodyne32.launch
   launch/fuser_velodyne32.launch
   launch/velo32.yaml
   launch/lamide_fuser_32.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY maps/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
