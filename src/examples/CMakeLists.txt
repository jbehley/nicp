add_executable(nicp_projective_aligner_example nicp_projective_aligner_example.cpp)
target_link_libraries(nicp_projective_aligner_example 
  map_core_library
  nicp_library
  ${catkin_LIBRARIES}
)

add_executable(nicp_spherical_aligner_example nicp_spherical_aligner_example.cpp)
target_link_libraries(nicp_spherical_aligner_example 
  map_core_library
  nicp_library
  ${catkin_LIBRARIES}
)

add_executable(nicp_3dscan2cloud_example nicp_3dscan2cloud_example.cpp)
target_link_libraries(nicp_3dscan2cloud_example 
  map_core_library
  nicp_library
  ${catkin_LIBRARIES}
)

add_executable(nicp_nn_aligner_example nicp_nn_aligner_example.cpp)
target_link_libraries(nicp_nn_aligner_example 
  map_core_library
  nicp_library
  ${catkin_LIBRARIES}
)

add_executable(trajectory_loader_example trajectory_loader_example.cpp)
target_link_libraries(trajectory_loader_example 
  map_core_library
  ${catkin_LIBRARIES}
)
