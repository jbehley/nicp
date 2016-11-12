add_executable(cloud_viewer_gui_app cloud_viewer_gui_app.cpp)
target_link_libraries(cloud_viewer_gui_app
  map_core_library
  map_viewers_library
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

add_executable(nicp_projective_aligner_gui_app nicp_projective_aligner_gui_app.cpp)
target_link_libraries(nicp_projective_aligner_gui_app
  map_core_library
  nicp_library
  map_viewers_library
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

add_executable(nicp_spherical_aligner_gui_app nicp_spherical_aligner_gui_app.cpp)
target_link_libraries(nicp_spherical_aligner_gui_app
  map_core_library
  nicp_library
  map_viewers_library
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

add_executable(nicp_nn_aligner_gui_app nicp_nn_aligner_gui_app.cpp)
target_link_libraries(nicp_nn_aligner_gui_app
  map_core_library
  nicp_library
  map_viewers_library
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

add_executable(trajectory_loader_gui_app trajectory_loader_gui_app.cpp)
target_link_libraries(trajectory_loader_gui_app 
  map_core_library
  map_viewers_library
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

add_executable(tracker_gui_app tracker_gui_app.cpp)
target_link_libraries(tracker_gui_app 
  map_core_library
  nicp_library
  map_tracker_library
  map_viewers_library
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

add_executable(local_mapper_gui_app local_mapper_gui_app.cpp)
target_link_libraries(local_mapper_gui_app 
  map_core_library
  nicp_library
  map_tracker_library
  map_viewers_library
  local_mapper_library
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

add_executable(map_refiner_gui_app map_refiner_gui_app.cpp)
target_link_libraries(map_refiner_gui_app
  map_core_library
  map_global_optimization_library
  nicp_library
  map_viewers_library
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
  ${g2o_catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
)	
