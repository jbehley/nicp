cmake_minimum_required(VERSION 2.8.3)
project(nicp_suite)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(ENABLE_ROS_NODES true)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
  # image_transport
  # sensor_msgs
  # nav_msgs
  # tf
  # message_generation
  # thin_state_publisher
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_BUILD_TYPE Release)

## System dependencies are found with CMake's conventions

# Find PkgConfig
# find_package(PkgConfig REQUIRED)

# Find Eigen3
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

# Find Boost
# find_package(Boost COMPONENTS system filesystem REQUIRED)


# Find Flann
# pkg_check_modules(FLANN flann REQUIRED)
# include(FindPackageHandleStandardArgs)
# find_package_handle_standard_args(FLANN DEFAULT_MSG FLANN_LIBRARY FLANN_INCLUDE_DIR)

# Find OpenMP
# find_package(OpenMP)
# if(OPENMP_FOUND)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_POSIT_SOLVER_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
#  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif(OPENMP_FOUND)

# Find OpenCV
# find_package(OpenCV)
# include_directories(${OpenCV_INCLUDE_DIRS})

# Find ZLib
# find_package(ZLIB REQUIRED)
# include_directories(${ZLIB_INCLUDE_DIRS})

# Set up the top-level include directories
# include_directories(${PROJECT_SOURCE_DIR}/src)

# Find OpenGL
# find_package(OpenGL REQUIRED)
# include_directories(${OPENGL_INCLUDE})

# Find Qt4
# find_package(Qt4 REQUIRED)
# include_directories(${QT_INCLUDES})

# Find QGLViewer
# find_package(QGLViewer REQUIRED)
# include_directories(${QGLVIEWER_INCLUDE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

#Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   RichPointMsg.msg
#   CloudMsg.msg
#   StampedCloudMsg.msg
#   TrajectoryNodeMsg.msg
#   MapNodeMsg.msg
#   LocalMapMsg.msg
#   BaseCameraInfoMsg.msg
#   PinholeCameraInfoMsg.msg
#   MultiCameraInfoMsg.msg
#   ImageMapNodeMsg.msg
#   MultiImageMapNodeMsg.msg
#   BinaryNodeRelationMsg.msg
#   MapNodeUpdateMsg.msg
#   MapUpdateMsg.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#    FILES
#    IdsSrv.srv  LocalMapByIdSrv.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES 
  # boss_library
  # txt_io_library
  # fps_core_library			
  # fps_tracker_library
  # fps_extrinsic_calibrator_library	
  # fps_map_library
  # fps_local_mapper_library
  # fps_ros_library
  # fps_ros_viewers_library
  # fps_global_optimization_library
  # fps_loop_closer2_library
  # fps_loop_closer_library
  # fps_gl_helpers_library
  # fps_core_viewers_library
  # fps_tracker_viewers_library
  # fps_map_viewers_library
  # fps_local_mapper_viewers_library
  # ros_wrappers_library

  CATKIN_DEPENDS message_runtime 
#  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${voronoi_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp library
# add_library(ptracker
#   src/${PROJECT_NAME}/ptracker.cpp
# )

## Declare a cpp executable
# add_executable(ptracker_node src/ptracker_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ptracker_node ptracker_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ptracker_node
#   ${catkin_LIBRARIES}
# )

# ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ptracker ptracker_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ptracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
