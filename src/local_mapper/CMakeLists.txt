add_library(local_mapper_library SHARED
  local_map_triggers.cpp local_map_triggers.h
)
target_link_libraries(local_mapper_library 
  map_core_library
  nicp_library
  map_tracker_library
)
