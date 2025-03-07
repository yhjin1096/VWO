cmake_minimum_required(VERSION 3.0.2)
project(VWO)

# set(g2o_DIR "${PROJECT_SOURCE_DIR}/thirdparty/g2o/install/lib/cmake/g2o")
find_package(g2o REQUIRED)
# get_target_property(_g2o_core_inc        g2o::core INTERFACE_INCLUDE_DIRECTORIES)
# get_target_property(_g2o_core_location  g2o::core IMPORTED_LOCATION_RELEASE) # 또는 IMPORTED_LOCATION
# message(STATUS "g2o::core include dirs: ${_g2o_core_inc}")
# message(STATUS "g2o::core library loc: ${_g2o_core_location}")

find_package(Eigen3 REQUIRED)

find_package(OpenCV)

find_package(yaml-cpp REQUIRED)

set(fbow_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/FBoW)
add_subdirectory(${fbow_DIRS} ${PROJECT_BINARY_DIR}/thirdparty/FBoW)
find_package(fbow QUIET)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_msgs
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES VWO
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters nav_msgs roscpp sensor_msgs tf2 tf2_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${fbow_INCLUDE_DIRS}
  /usr/include/suitesparse #sudo apt-get install libsuitesparse-dev
)

add_executable(${PROJECT_NAME}_node 
              src/camera/camera.cpp
              src/camera/perspective.cpp

              src/data/bow_database.cc
              src/data/bow_vocabulary.cc
              src/data/common.cpp
              src/data/camera_database.cc
              src/data/frame_statistics.cc
              src/data/frame.cpp
              src/data/graph_node.cc
              src/data/keyframe.cpp
              src/data/landmark.cpp
              src/data/map_database.cc
              src/data/orb_params_database.cc

              src/feature/feature_extractor.cpp
              src/feature/orb_extractor.cpp
              src/feature/orb_params.cpp
              src/feature/orb_impl.cpp

              src/initialize/base.cpp
              src/initialize/perspective.cpp

              src/match/area.cpp

              src/module/initializer.cpp

              src/optimize/global_bundle_adjuster.cc
              src/optimize/terminate_action.cc

              src/solve/common.cpp
              src/solve/essential_solver.cpp
              src/solve/homography_solver.cpp
              src/solve/fundamental_solver.cpp

              src/tracking/tracking_module.cpp

              src/util/converter.cpp
              src/util/angle.cpp
              src/util/random_array.cpp
              src/util/sqlite3.cc

              src/main.cpp
              src/VWO_node.cpp
              src/system.cpp
              src/config.cpp
              )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  ${fbow_LIBS}
  sqlite3
  g2o::core
  g2o::stuff
  g2o::types_sba
  g2o::types_sim3
  g2o::solver_dense
  g2o::solver_eigen
  g2o::solver_csparse
  g2o::csparse_extension
)