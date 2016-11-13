add_executable(tracker_node tracker_node.cpp)
target_link_libraries(tracker_node   
  nicp_library
  map_tracker_library
  map_viewers_library
  map_ros_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(tracker_viewer_node  
  tracker_viewer_node.cpp
)
target_link_libraries(tracker_viewer_node 
  map_tracker_library
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

add_executable(local_mapper_node local_mapper_node.cpp)
target_link_libraries(local_mapper_node 
  map_tracker_library
  map_ros_library
  map_viewers_library    
  ${catkin_LIBRARIES}
)

add_executable(global_mapper_viewer_node global_mapper_viewer_node.cpp)
target_link_libraries(global_mapper_viewer_node 
  map_viewers_ros_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(local_mapper_viewer_node local_mapper_viewer_node.cpp)
target_link_libraries(local_mapper_viewer_node 
  map_viewers_ros_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(local_mapper_client_node_example local_mapper_client_node_example.cpp)
target_link_libraries(local_mapper_client_node_example 
  map_ros_library
  ${catkin_LIBRARIES}
)

add_executable(local_mapper_dumper_node local_mapper_dumper_node.cpp)
target_link_libraries(local_mapper_dumper_node
  map_ros_library
  ${catkin_LIBRARIES}
)

add_executable(map_refiner_node 
  map_refiner_node.cpp
)
target_link_libraries(map_refiner_node
  map_viewers_ros_library
  map_ros_library
  map_global_optimization_library
  map_core_library 
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${CSPARSE_LIBRARY}
)
