add_library(nicp_globals SHARED
  base_image_message.cpp base_image_message.h
  base_message.cpp base_message.h
  base_sensor_message.cpp base_sensor_message.h
  message_factory.cpp message_factory.h
  message_reader.cpp message_reader.h
  message_seq_synchronizer.cpp message_seq_synchronizer.h  
  message_writer.cpp message_writer.h
  opengl_primitives.cpp opengl_primitives.h
  pinhole_image_message.cpp pinhole_image_message.h
  sensor_message_sorter.cpp sensor_message_sorter.h
  simple_viewer.cpp simple_viewer.h
  spherical_image_message.cpp spherical_image_message.h
  static_transform_message.cpp static_transform_message.h
  static_transform_tree.cpp static_transform_tree.h
  system_utils.cpp system_utils.h
)
target_link_libraries(nicp_globals
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
