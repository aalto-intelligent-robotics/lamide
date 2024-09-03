cmake_minimum_required(VERSION 2.8.3)
project(ndt_rviz_lamide)
set(CMAKE_BUILD_TYPE release)
find_package(catkin REQUIRED COMPONENTS
  rviz
  ndt_map_lamide
  cmake_modules
  eigen_conversions
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

find_package(Qt5Widgets REQUIRED)
set(CMAKE_CXX_STANDARD 11)

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS rviz ndt_map_lamide tf_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

qt5_wrap_cpp(MOC_FILES
    include/ndt_rviz_lamide/ndt_display.hpp
    include/ndt_rviz_lamide/ndt_rgb_display.hpp
    include/ndt_rviz_lamide/ndt_line_display.hpp
    include/ndt_rviz_lamide/ndt_vector_display.hpp
)

# # Here we specify the list of source files, including the output of
# # the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
    src/ndt_display.cpp
    src/ndt_rgb_display.cpp
    src/ndt_visual.cpp
    src/ndt_line_display.cpp
    src/ndt_line_visual.cpp
    src/ndt_vector_display.cpp
    src/ndt_rviz_lamide.cpp
    src/utils.cpp
    ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

## Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# # Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

#install(DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#install(PROGRAMS scripts/send_test_msgs.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

