add_library(velodyne_rawdata_oru rawdata.cc calibration.cc ../conversions/pointcloudXYZIR.cc)
target_link_libraries(velodyne_rawdata_oru
                      ${catkin_LIBRARIES}
                      ${YAML_CPP_LIBRARIES})
install(TARGETS velodyne_rawdata_oru
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
