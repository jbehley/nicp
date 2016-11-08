add_library(nicp_viewers SHARED
  aligner_viewer.cpp aligner_viewer.h
  cloud_viewer.cpp cloud_viewer.h
  local_mapper_viewer.cpp local_mapper_viewer.h
  tracker_viewer.cpp tracker_viewer.h
  trajectory_viewer.cpp trajectory_viewer.h
)
target_link_libraries(nicp_viewers
  nicp_local_mapper
  nicp_map
  nicp_tracker
  nicp_core 
  nicp_boss
  nicp_globals
  ${FLANN_LIBRARIES}
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  ${catkin_LIBRARIES}
)
