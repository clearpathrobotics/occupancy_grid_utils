cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_init()
rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_library(grid_utils src/ray_tracer.cpp
                                src/grid_overlay.cpp
                                src/shortest_path.cpp
                                src/coordinate_conversions.cpp
                                src/combine_grids.cpp
                                src/geometry.cpp
                                src/file.cpp)
target_link_libraries(grid_utils SDL SDL_image)

rosbuild_add_gtest(test_grid_utils test/test_grid_utils.cpp)
target_link_libraries(test_grid_utils grid_utils)

rosbuild_add_executable(grid_construction_node
                        src/examples/grid_construction_node.cpp)
target_link_libraries(grid_construction_node
                      grid_utils)


# Python stuff

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})                    

rosbuild_add_library(occupancy_grid_utils src/boost_python_exports.cpp)
target_link_libraries(occupancy_grid_utils grid_utils)
target_link_libraries(occupancy_grid_utils ${PYTHON_LIBRARIES})
rosbuild_link_boost(occupancy_grid_utils python)

set_target_properties(occupancy_grid_utils
  PROPERTIES
  PREFIX "")

rosbuild_make_distribution(0.5.1)