add_library(nicp_map SHARED
  binary_node_relation.cpp binary_node_relation.h
  image_map_node.cpp image_map_node.h
  local_map.h local_map.cpp
  map_node.cpp map_node.h
  map_node_list.cpp map_node_list.h
  multi_image_map_node.cpp multi_image_map_node.h  
)
target_link_libraries(nicp_map 
  nicp_core 
  nicp_boss
  nicp_globals
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)


