cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization_lamide)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp ndt_map_lamide sensor_msgs geometry_msgs laser_geometry message_filters
  velodyne_pointcloud_oru tf_conversions ndt_generic_lamide
  )
# set(CMAKE_CXX_FLAGS "-std=c++11 -ggdb ${CMAKE_CXX_FLAGS}")
# find_package(OpenMP)

set (CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#set(CMAKE_BUILD_TYPE Release)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES g_ndt_mcl_lamide
        CATKIN_DEPENDS ndt_map_lamide roscpp sensor_msgs geometry_msgs laser_geometry message_filters ndt_generic_lamide
        velodyne_pointcloud_oru
        DEPENDS
        LIBRARIES ${PROJECT_NAME}
)




###########
## Build ##
###########

include_directories(include
   ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/particle.cpp
        src/${PROJECT_NAME}/particle_filter.cpp
        src/${PROJECT_NAME}/particle_filter_HMT.cpp
        src/${PROJECT_NAME}/helper_functions.cpp
        src/${PROJECT_NAME}/ndt_histogram_2D_lamide.cpp
        src/${PROJECT_NAME}/3d_ndt_mcl_lamide.cpp
        src/${PROJECT_NAME}/3d_particle_filter.cpp
        )





#add_definitions(${EIGEN_DEFINITIONS})


#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
# message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()


add_executable(ndt_mcl_localization_lamide src/ndt_mcl_localization_lamide.cpp)
#add_executable(ndt_mcl_localization_filter_lamide src/ndt_mcl_localization_filter_lamide.cpp)
add_executable(ndt_map_publisher_lamide src/ndt_map_publisher_lamide.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(ndt_mcl_localization_lamide
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#target_link_libraries(ndt_mcl_localization_filter_lamide
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#)

target_link_libraries(ndt_map_publisher_lamide
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ndt_map_publisher_lamide ndt_mcl_localization_lamide #ndt_mcl_localization_filter_lamide
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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
