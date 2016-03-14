# build the driver node
add_executable(o3m151_node o3m151_node.cc driver.cc)
target_link_libraries(o3m151_node
  o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(o3m151driver_nodelet nodelet.cc driver.cc)
target_link_libraries(o3m151driver_nodelet
  o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS o3m151_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS o3m151driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
