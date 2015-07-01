# build the driver node
#message(STATUS "PROJECT_NAME " "^${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/")
file(GLOB_RECURSE include_files "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/*")
file(GLOB_RECURSE launch_files "${CMAKE_SOURCE_DIR}/launch/*")
list(APPEND extra_files ${include_files} ${launch_files})
#message(STATUS "extra_files " ${extra_files})

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(o3m151_node o3m151_node.cc driver.cc ${extra_files})
target_link_libraries(o3m151_node
  o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
  ${PCL_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
target_link_libraries(driver_nodelet
  o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
  ${PCL_LIBRARIES}
)

# install runtime files
install(TARGETS o3m151_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
