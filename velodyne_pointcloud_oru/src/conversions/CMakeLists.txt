add_executable(cloud_node_oru cloud_node.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(cloud_node_oru ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_node_oru velodyne_rawdata_oru
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_node_oru
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(cloud_nodelet_oru cloud_nodelet.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(cloud_nodelet_oru ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_nodelet_oru velodyne_rawdata_oru
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_nodelet_oru
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(transform_node_oru transform_node.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(transform_node_oru ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node_oru velodyne_rawdata_oru
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_node_oru
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(transform_nodelet_oru transform_nodelet.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(transform_nodelet_oru ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet_oru velodyne_rawdata_oru
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_nodelet_oru
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
