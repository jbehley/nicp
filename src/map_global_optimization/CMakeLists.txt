set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11")

add_library(map_global_optimization_library SHARED
  g2o_bridge.cpp g2o_bridge.h
)
target_link_libraries(map_global_optimization_library
  map_core_library
  ${g2o_catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
)
