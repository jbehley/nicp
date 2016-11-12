add_library(map_ros_library
  cloud_publisher_trigger.cpp cloud_publisher_trigger.h
  local_map_listener.cpp local_map_listener.h
  local_mapper_ros.cpp local_mapper_ros.h
  map_msgs_ros.cpp map_msgs_ros.h
)
target_link_libraries(map_ros_library
  map_core_library
  nicp_library
  map_tracker_library
  local_mapper_library
  ${catkin_LIBRARIES}
)
