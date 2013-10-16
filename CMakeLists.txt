cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS 
    sensor_msgs laser_geometry cv_bridge 
    tf geometry_msgs pcl_ros pcl_conversions 
    tf2_bullet roscpp std_msgs message_generation
    message_runtime nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system signals python)
find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Bullet REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  OverlayClouds.msg
  NavigationFunction.msg
  LocalizedCloud.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_utils occupancy_grid_utils
  CATKIN_DEPENDS nav_msgs geometry_msgs sensor_msgs laser_geometry roscpp rospy map_server tf
  DEPENDS system_lib
)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_PATH})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIR})
link_libraries(${catkin_LIBRARIES})
link_libraries(${PYTHON_LIBRARIES})

# Build
add_library(grid_utils 
    src/ray_tracer.cpp
    src/grid_overlay.cpp
    src/shortest_path.cpp
    src/coordinate_conversions.cpp
    src/combine_grids.cpp
    src/geometry.cpp
    src/file.cpp
)
target_link_libraries(grid_utils SDL SDL_image)

add_executable(grid_construction_node
    src/examples/grid_construction_node.cpp)
target_link_libraries(grid_construction_node
    grid_utils)

# Python stuff
add_library(occupancy_grid_utils src/boost_python_exports.cpp)
target_link_libraries(occupancy_grid_utils grid_utils)
set_target_properties(occupancy_grid_utils
  PROPERTIES
  PREFIX "")

