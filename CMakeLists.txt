cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS 
    nav_msgs geometry_msgs sensor_msgs laser_geometry
    roscpp rospy tf tf2_bullet message_generation
)

## Add find-modules and define external dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED COMPONENTS system signals python)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Bullet REQUIRED)
find_package(OpenCV REQUIRED)

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
  LIBRARIES grid_utils
  CATKIN_DEPENDS nav_msgs geometry_msgs sensor_msgs laser_geometry roscpp rospy tf tf2_bullet message_runtime
)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_PATH})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

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
add_dependencies(grid_utils occupancy_grid_utils_generate_messages_cpp)
target_link_libraries(grid_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(grid_construction_node
    src/examples/grid_construction_node.cpp)
add_dependencies(grid_construction_node occupancy_grid_utils_generate_messages_cpp)
target_link_libraries(grid_construction_node
    grid_utils ${catkin_LIBRARIES})

# Python stuff
add_library(grid_utils_boost_python_exports src/boost_python_exports.cpp)
target_link_libraries(grid_utils_boost_python_exports grid_utils ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS grid_construction_node grid_utils grid_utils_boost_python_exports 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h**"
  PATTERN ".svn" EXCLUDE
)

