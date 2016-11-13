add_library(map_viewers_ros_library
  global_map_viewer.cpp global_map_viewer.h
  local_map_viewer.cpp local_map_viewer.h
)
target_link_libraries(map_viewers_ros_library
  map_core_library
  map_viewers_library
  map_ros_library  
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${catkin_LIBRARIES}
)
