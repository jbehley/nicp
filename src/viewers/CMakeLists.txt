add_library(map_viewers_library SHARED
  aligner_viewer.cpp aligner_viewer.h
  cloud_viewer.cpp cloud_viewer.h
  local_mapper_viewer.cpp local_mapper_viewer.h
  tracker_viewer.cpp tracker_viewer.h
  trajectory_viewer.cpp trajectory_viewer.h
)
target_link_libraries(map_viewers_library
  map_core_library
  nicp_library
  map_tracker_library
  local_mapper_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
