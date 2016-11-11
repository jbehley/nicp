set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11")

add_library(nicp_global_optimization SHARED
  g2o_bridge.cpp g2o_bridge.h
)
target_link_libraries(nicp_global_optimization
  nicp_core 
  nicp_map
  nicp_boss
  nicp_globals
  ${g2o_LIBRARIES}
  ${CSPARSE_LIBRARY}
)
