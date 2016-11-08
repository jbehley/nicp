add_library(nicp_core SHARED 
  base_aligner.cpp base_aligner.h
  base_camera_info.cpp base_camera_info.h
  base_correspondence_finder.cpp base_correspondence_finder.h
  base_projector.cpp base_projector.h
  camera_info_manager.cpp camera_info_manager.h
  cloud.cpp cloud.h
  depth_utils.cpp depth_utils.h
  multi_camera_info.cpp multi_camera_info.h
  multi_projector.cpp multi_projector.h
  nn_aligner.cpp nn_aligner.h
  nn_correspondence_finder.cpp nn_correspondence_finder.h
  pinhole_camera_info.cpp pinhole_camera_info.h
  pinhole_projector.cpp pinhole_projector.h
  projective_aligner.cpp projective_aligner.h
  projective_correspondence_finder.cpp projective_correspondence_finder.h
  spherical_camera_info.cpp spherical_camera_info.h
  solver.cpp solver.h
  spherical_projector.cpp spherical_projector.h
)

target_link_libraries(nicp_core 
  nicp_boss
  nicp_globals
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  ${FLANN_LIBRARIES} 
  ${catkin_LIBRARIES}
)
