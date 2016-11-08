add_library(nicp_local_mapper SHARED
  local_map_triggers.cpp local_map_triggers.h
)
target_link_libraries(nicp_local_mapper 
  nicp_map
  nicp_tracker
  nicp_core 
  nicp_boss
  nicp_globals
  ${FLANN_LIBRARIES}
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)
