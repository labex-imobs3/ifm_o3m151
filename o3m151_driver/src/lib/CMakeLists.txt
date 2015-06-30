add_library(o3m151_input input.cc)
target_link_libraries(o3m151_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(o3m151_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS o3m151_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
