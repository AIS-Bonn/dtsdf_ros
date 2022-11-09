cmake_minimum_required(VERSION 3.0.2)
project(dtsdf_ros)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_ros)

set(InfiniTAM_BUILD_APPS OFF)
set(InfiniTAM_BUILD_TESTS OFF)
add_subdirectory(DirectionalTSDF)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dtsdf_ros
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters pcl_ros  roscpp sensor_msgs tf2_eigen tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(dtsdf_node src/main.cpp src/Mapper.cpp)
set_target_properties(dtsdf_node PROPERTIES OUTPUT_NAME dtsdf_node PREFIX "")
add_dependencies(dtsdf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dtsdf_node
  ${catkin_LIBRARIES}
  ITMLib::ITMLib
)