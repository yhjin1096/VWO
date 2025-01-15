cmake_minimum_required(VERSION 3.0.2)
project(VWO)

set(G2O_ROOT ${CMAKE_HOME_DIRECTORY}/third_party/g2o)
# add_subdirectory(${G2O_ROOT})

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)