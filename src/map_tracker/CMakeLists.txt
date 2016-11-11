add_library(map_tracker_library SHARED
  base_triggers.cpp  base_triggers.h
  call_tracker_trigger.h call_tracker_trigger.cpp
  multi_tracker.cpp  multi_tracker.h
  tracker.cpp  tracker.h  
)
target_link_libraries(map_tracker_library 
  map_core_library
  nicp_library
)
