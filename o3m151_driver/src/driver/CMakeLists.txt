# build the driver node
#message(STATUS "PROJECT_NAME " "^${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/")
file(GLOB_RECURSE include_files "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/*")
#message(STATUS "include_files " ${include_files})

add_executable(o3m151_node o3m151_node.cc driver.cc ${include_files})
target_link_libraries(o3m151_node
  o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
target_link_libraries(driver_nodelet
  o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS o3m151_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
