add_executable(nicp_projective_aligner_test nicp_projective_aligner_test.cpp)
target_link_libraries(nicp_projective_aligner_test 
  nicp_boss
  nicp_core 
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

add_executable(nicp_spherical_aligner_test nicp_spherical_aligner_test.cpp)
target_link_libraries(nicp_spherical_aligner_test 
  nicp_boss
  nicp_core 
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

add_executable(nicp_3dscan2cloud_test nicp_3dscan2cloud_test.cpp)
target_link_libraries(nicp_3dscan2cloud_test 
  nicp_boss
  nicp_core 
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

add_executable(nicp_nn_aligner_test nicp_nn_aligner_test.cpp)
target_link_libraries(nicp_nn_aligner_test 
  nicp_boss
  nicp_core 
  ${OpenCV_LIBS} 
  ${FLANN_LIBRARIES} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)
