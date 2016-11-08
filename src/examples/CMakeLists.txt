add_executable(nicp_projective_aligner_example nicp_projective_aligner_example.cpp)
target_link_libraries(nicp_projective_aligner_example 
  nicp_boss
  nicp_core 
)

add_executable(nicp_spherical_aligner_example nicp_spherical_aligner_example.cpp)
target_link_libraries(nicp_spherical_aligner_example 
  nicp_boss
  nicp_core 
)

add_executable(nicp_3dscan2cloud_example nicp_3dscan2cloud_example.cpp)
target_link_libraries(nicp_3dscan2cloud_example 
  nicp_boss
  nicp_core 
)

add_executable(nicp_nn_aligner_example nicp_nn_aligner_example.cpp)
target_link_libraries(nicp_nn_aligner_example 
  nicp_boss
  nicp_core 
)

add_executable(nicp_trajectory_loader_example nicp_trajectory_loader_example.cpp)
target_link_libraries(nicp_trajectory_loader_example 
  nicp_map
  nicp_core 
  nicp_boss
  nicp_globals
)
