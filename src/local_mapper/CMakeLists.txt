add_library(local_mapper_library SHARED
  local_mapper.cpp local_mapper.h
)
target_link_libraries(local_mapper_library 
  map_core_library
  nicp_library
  map_tracker_library
)
