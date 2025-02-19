cmake_minimum_required(VERSION 3.0.2)
project(VWO)

set(G2O_ROOT ${CMAKE_HOME_DIRECTORY}/third_party/g2o)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/thirdparty/g2o/cmake_modules)

set(G2O_LIBS_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/g2o/build/lib)
set(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/g2o)

find_package(Eigen3 REQUIRED)

find_package(OpenCV)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES VWO
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters nav_msgs roscpp sensor_msgs tf2 tf2_msgs
#  DEPENDS system_lib
)

link_directories(${G2O_LIBS_DIRS})

set(G2O_LIBS
    g2o_core 
    g2o_types_sba 
    g2o_types_slam3d
    g2o_solver_csparse 
    g2o_stuff 
    g2o_csparse_extension
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
              src/main.cpp
              
              src/VWO_node.cpp
              src/system.cpp

              src/config.cpp

              src/camera/camera.cpp
              src/camera/perspective.cpp

              src/data/common.cpp
              src/data/frame.cpp
              src/data/keyframe.cpp
              src/data/landmark.cpp

              src/feature/feature_extractor.cpp
              src/feature/orb_extractor.cpp
              src/feature/orb_params.cpp
              src/feature/orb_impl.cpp

              src/initialize/base.cpp
              src/initialize/perspective.cpp

              src/match/area.cpp

              src/solve/common.cpp
              src/solve/essential_solver.cpp
              src/solve/homography_solver.cpp
              src/solve/fundamental_solver.cpp

              src/tracking/tracking_module.cpp

              src/util/converter.cpp
              src/util/angle.cpp
              src/util/random_array.cpp
              )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)