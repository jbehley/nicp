add_library(nicp_tracker SHARED
  base_triggers.cpp  base_triggers.h
  call_tracker_trigger.h call_tracker_trigger.cpp
  multi_tracker.cpp  multi_tracker.h
  tracker.cpp  tracker.h  
)
target_link_libraries(nicp_tracker 
  nicp_core 
  nicp_boss
  ${FLANN_LIBRARIES}
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)
